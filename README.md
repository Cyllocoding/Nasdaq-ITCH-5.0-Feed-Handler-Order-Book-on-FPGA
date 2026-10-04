# FPGA Nasdaq ITCH 5.0 Feed Handler & Order Book

A low-latency, deterministic market-data pipeline on an **AMD Kria KR260** (Zynq UltraScale+ XCK26), written in **SystemVerilog** and built with **Vivado**.

It receives a Nasdaq TotalView-ITCH 5.0 feed over 10G Ethernet (MoldUDP64/UDP multicast), tracks every live order for a configurable set of stocks, and emits the **best bid / best ask (BBO)** of each stock whenever it changes — with a fixed-latency fast path of roughly **130 ns from last byte on the wire to BBO update** on an L1 hit.

```
 SFP+ 10G ──► MAC/PCS ──► [1: Feed handler / parser] ──► normalised order events
                                                              │
                                                              ▼
                                        [2: Order memory — L1 URAM cache + L2 DDR]
                                                              │  {book, side, price, Δqty}
                                                              ▼
                                        [3: Price book + best-price tracking]
                                                              │
                                                              ▼
                                               BBO updates ──► AXI-Stream / registers ──► ARM (PS)
```

---

## Table of contents

1. [Why this project](#1-why-this-project)
2. [Platform and global decisions](#2-platform-and-global-decisions)
3. [Block 1 — Feed handler / parser](#3-block-1--feed-handler--parser)
4. [Block 2 — Order memory (hash table, L1 + L2)](#4-block-2--order-memory-hash-table-l1--l2)
5. [Block 3 — Price book and best-price tracking](#5-block-3--price-book-and-best-price-tracking)
6. [Control plane and software](#6-control-plane-and-software)
7. [Resource budget](#7-resource-budget)
8. [Latency budget](#8-latency-budget)
9. [Verification](#9-verification)
10. [Build milestones](#10-build-milestones)
11. [What I'd do differently / extensions](#11-what-id-do-differently--extensions)

---

## 1. Why this project

Exchange feeds are a textbook FPGA problem: the data arrives at line rate, it cannot be paused, and the value of a result decays with every nanosecond. A software order book is easy to write but its latency is neither low nor predictable. The point of doing it in hardware is **determinism**: every operation on the fast path takes a fixed number of cycles, and anything whose latency is variable (external DRAM) is kept off the fast path or made to stall explicitly.

The three sub-problems I had to solve are:

1. **Parsing at line rate** — ITCH messages are variable-length, back to back, and start at arbitrary byte lanes of a 64-bit bus.
2. **Remembering every live order** — Execute / Cancel / Delete / Replace messages only carry an order ID, so I need an `order_id → {side, price, qty}` lookup that is fast, large, and gracefully handles collisions.
3. **Finding the best price** — when the best level empties, the next-best level must be found in a bounded number of cycles.

---

## 2. Platform and global decisions

| Topic | Decision | Why / what else I considered |
|---|---|---|
| Board | **Kria KR260** — XCK26: 64 URAM, 144 BRAM36, ~256K logic cells, 1× SFP+, 4 GB DDR4 on the PS | Has URAM (essential for the order table), a 10G cage, and the same toolchain as industry (Vivado, UltraScale+). |
| HDL | **SystemVerilog** | `struct packed`, `typedef enum`, `always_ff` make the datapath readable and self-documenting. |
| Line rate | **10GBASE-R** | The only SFP+ rate practical at home (PC 10G NIC + DAC cable). |
| Datapath | **64-bit AXI-Stream at 156.25 MHz** (6.4 ns/cycle) | Native 10G rate: 8 B × 156.25 MHz = 10 Gb/s. One beat per cycle, no rate conversion. |
| Clock domains | **Single clock domain** (the MAC RX clock) for the whole pipeline | No clock-domain crossing on the fast path → lower latency, simpler timing. URAM/BRAM close easily at 156 MHz. |
| Stocks tracked | **8 books** (`book_idx` = 3 bits) | Enough to be interesting; scaling is just widening `book_idx`. |
| ARM (PS) role | Configuration, symbol subscription, gap recovery, logging, counters | Never on the fast path. Slow or complex logic belongs in software. |
| Determinism | Every fast-path operation has fixed latency | The reason to use an FPGA at all. |

### What if I'd chosen differently?

- **Faster core clock (250–322 MHz) behind an async FIFO.** Each pipeline stage would be shorter (3–4 ns instead of 6.4 ns), so the ~20-cycle fast path would shrink to ~60–80 ns of compute. But the CDC crossing costs 2–4 cycles of its own, timing is harder to close in URAM/BRAM paths, and the parser still has to consume 64 bits per MAC cycle anyway. For v1 the single-domain simplicity wins; the faster core is a listed extension.
- **Wider datapath (e.g. 128-bit at 78 MHz).** Halves the beat rate but *two* messages could now start in one beat (smallest block is 14 bytes < 16), which breaks the "≤ 1 message per cycle" property that sizes every downstream block. Not worth it at 10G.
- **More books.** Each extra bit of `book_idx` doubles the price-level array (16 BRAM → 32 BRAM) and the occupancy bitmaps (16K → 32K flip-flops). 32 books would still fit comfortably.

---

## 3. Block 1 — Feed handler / parser

### 3.1 Function

Turn raw Ethernet frames into **normalised order events** for subscribed stocks, in sequence, without gaps. Output is at most **one event per cycle**:

```systemverilog
typedef enum logic [2:0] {EV_ADD, EV_EXEC, EV_CANCEL, EV_DELETE, EV_REPLACE} ev_op_t;

typedef struct packed {
  ev_op_t      op;
  logic [2:0]  book_idx;
  logic [63:0] order_id;      // original ID for REPLACE
  logic [63:0] new_order_id;  // REPLACE only
  logic        side;          // ADD only (0 = buy, 1 = sell); REPLACE inherits from old order
  logic [31:0] price;         // ADD, REPLACE (raw ITCH: 1/10000 $)
  logic [31:0] qty;           // ADD/REPLACE: shares; EXEC/CANCEL: shares removed
  logic [47:0] timestamp;     // ITCH ns since midnight
  logic [63:0] seq;           // MoldUDP64 sequence number of this message
} order_event_t;
```

### 3.2 MAC / PCS

I use an existing 10G MAC+PCS rather than writing one — either the AMD **10G/25G Ethernet Subsystem** IP or the open-source **verilog-ethernet** (Alex Forencich) 10G MAC/PHY, which supports UltraScale GTH transceivers and sidesteps the IP-licensing question. The MAC runs in **cut-through** mode: frames are passed beat by beat, and the FCS result is only known at `tlast`. (See 3.7 for what that implies.)

AXI-Stream input: `tdata[63:0]`, `tkeep[7:0]`, `tvalid`, `tlast`, `tuser` (bad FCS, valid with `tlast`). Frame byte *n* lives in beat *n/8*, lane *n%8*, with the first wire byte in `tdata[7:0]`.

### 3.3 Packet layout (no VLAN)

| Layer | Bytes | Frame offset |
|---|---|---|
| Ethernet (dst 6, src 6, ethertype 2) | 14 | 0–13 |
| IPv4 (IHL = 5) | 20 | 14–33 |
| UDP | 8 | 34–41 |
| MoldUDP64 header: session (10), sequence number (8), message count (2) | 20 | 42–61 |
| Message block: length (2) + ITCH message | 2 + len | from 62 |

The first message length field is bytes 62–63 (beat 7, lanes 6–7); the first ITCH type byte is byte 64 = beat 8, lane 0. Every message after that starts at an arbitrary lane.

Edge cases I handle:
- **VLAN tag** (ethertype 0x8100) shifts everything by 4 bytes → a 1-bit "offset +4" flag set after the ethertype check.
- **IPv4 options (IHL ≠ 5)** and **IP fragments** → drop and count. Exchange feeds never use them.
- **Endianness** — all ITCH and MoldUDP64 fields are big-endian, so bytes are swapped when assembled from AXI lanes.

### 3.4 Filtering (drop as early as possible)

| Filter | Rule |
|---|---|
| Ethertype | IPv4 (0x0800), optionally after one VLAN tag |
| Destination MAC / IP | match the configured multicast group (AXI-Lite registers) |
| IP protocol | UDP (17) only |
| UDP destination port | match configured port |
| MoldUDP64 session | match the configured 10-byte session ID |
| Stock (per message) | stock locate → book table (3.6); unsubscribed stocks dropped |

### 3.5 Message assembly — the core design decision of the parser

**Problem:** messages are back to back, start at any byte lane, and span several 64-bit beats.

**Key fact that sizes everything downstream:** the smallest ITCH 5.0 message is 12 bytes, so the smallest message *block* (with its 2-byte length prefix) is 14 bytes > 8. Therefore **at most one message can start per beat**, and every downstream block only needs to handle **≤ 1 message per cycle**.

I evaluated four approaches:

| Option | Description | Verdict |
|---|---|---|
| A. Byte-serial | Convert to an 8-bit stream, 1 byte/cycle | ❌ 8 bits × 156.25 MHz = 1.25 Gb/s. Can't keep up with 10 Gb/s. |
| B. Realign to lane 0 | Barrel shifter + carry register, emit aligned 64-bit beats per message | ❌ Each message gets padded up to whole beats. A 21-byte Delete block is 2.625 beats in but 3 beats out — the output rate exceeds the input rate for small back-to-back messages, so it overruns at line rate. |
| C. Extract fields at variable offsets | A mux per field per possible lane | Lowest latency, but the most complex and least maintainable. |
| **D. Message assembler** | Write incoming bytes into a **40-byte (320-bit) message register** at position `byte_count + lane`; when the message's last byte arrives, emit the whole message as one wide word | ✅ **Chosen** |

**Why D:** it guarantees ≤ 1 message per cycle (keeps line rate), decoding becomes trivial fixed bit-slices of a 320-bit word, and the latency cost is essentially zero because the fields I need in ITCH are near the *end* of the message anyway (price is the last field of Add; order ID is the last field of Delete) — I'd have to wait for the last beat regardless.

Implementation details:
- State per packet: `in_header`, `msg_len` (from the 2-byte prefix), `msg_byte_cnt`, `msgs_remaining` (from the MoldUDP64 count).
- **Two message registers (ping-pong)** because one beat can contain the end of message *k* and the start of message *k+1*.
- Messages longer than 40 bytes (NOII, some trade messages): store the first 40 bytes, count the rest via the length field, skip.
- Each of the 40 byte positions is fed by an 8:1 mux from the 8 lanes → 320 × 8:1 mux bits. Cheap.
- `tlast` arriving before the expected byte count → drop the partial message, count the error, treat as a gap.

> **What if the register were smaller?** 36 bytes would cover Add (36) but not Add-with-MPID (40), which would then need the "skip the tail" path even though it's an order message I need. 40 bytes covers every order-book-relevant message exactly. Going larger buys nothing.

### 3.6 Symbol filter and book mapping

Stock locate codes are **assigned fresh every day** and published in **Stock Directory ('R')** messages at the start of the feed. So the hardware compares the 8-byte symbol in each 'R' message against up to 8 subscribed symbols (8 × 64-bit comparators, symbols written by the PS) and writes `locate → {valid, book_idx}` into a table. The PS can also write the table directly, which is needed for replays that start mid-day.

- Table: 65,536 locates × 4 bits = 256 Kbit → **8 BRAM36**.
- Read with the 2-byte locate from every message; the 1–2 cycle latency overlaps with message assembly, so it adds nothing to the critical path.

### 3.7 Messages decoded

Offsets are from the type byte (offset 0). Common header: type [0], stock locate [1–2], tracking number [3–4], timestamp [5–10].

| Type | Name | Length | Fields used | Event |
|---|---|---|---|---|
| `A` | Add Order | 36 | order ref [11–18], side [19] ('B'/'S'), shares [20–23], stock [24–31], price [32–35] | ADD |
| `F` | Add Order with MPID | 40 | same as A (attribution [36–39] ignored) | ADD |
| `E` | Order Executed | 31 | order ref [11–18], executed shares [19–22] | EXEC |
| `C` | Order Executed with Price | 36 | order ref [11–18], executed shares [19–22] (exec price ignored for the book) | EXEC |
| `X` | Order Cancel | 23 | order ref [11–18], cancelled shares [19–22] | CANCEL |
| `D` | Order Delete | 19 | order ref [11–18] | DELETE |
| `U` | Order Replace | 35 | original ref [11–18], new ref [19–26], shares [27–30], price [31–34] | REPLACE |
| `R` | Stock Directory | — | symbol, locate | symbol table only |
| others | system events, trades, halts, NOII, … | — | — | skipped by length |

Two subtleties:
- **`C` (Executed with Price):** the book is reduced at the order's **resting** price (stored in order memory), not at the execution price.
- **`U` (Replace):** carries **no side field** — the new order inherits the side of the original. This has to be resolved in Block 2 where the old order's side is stored.

### 3.8 Sequencing, gaps and bad frames

MoldUDP64's header sequence number is the sequence of the *first* message in the packet; message *i* has `seq + i`. The parser keeps an `expected_seq` register:

| Condition | Action |
|---|---|
| `seq == expected` | accept; `expected += msg_count` |
| `seq + count <= expected` | full duplicate → drop |
| `seq < expected < seq + count` | partial duplicate → skip the first `expected − seq` messages |
| `seq > expected` | **gap** → set `book_stale`, raise interrupt, count |
| `count == 0` | heartbeat → nothing to do, no seq advance |
| `count == 0xFFFF` | end of session |

**Gap recovery is done in software.** Hardware only detects and flags; the book is marked stale until the PS clears it (after requesting retransmission/snapshot from a recovery service, or in a replay setup simply restarting).

**Bad FCS at `tlast`:** because the MAC is cut-through, events from the frame were already emitted before I learn the frame is bad. Decision: treat a bad frame as a **gap** (stale flag + recovery). Corruption is rare on a DAC cable, and this keeps zero added latency.
*Alternative:* hold a packet's events in a small FIFO until `tlast` (store-and-forward at event granularity). Always correct, but adds up to one full packet of latency to every message — a bad trade for something that almost never happens.

**A/B arbitration (extension):** exchanges send two copies of the feed on lines A and B. With one `expected_seq`, forward whichever copy arrives first and drop the other as a duplicate; same-cycle arrival → fixed priority to A. Both lines must be in the same clock domain (async FIFO if not). The KR260 has one SFP+, so this is verified in simulation (or with a second 1G port as line B).

### 3.9 Backpressure

**The network cannot be backpressured.** The parser never stalls; it pushes events into an **input event FIFO of 512 entries** in front of Block 2. If the FIFO fills, events are dropped and `fifo_overflow` is raised — which is semantically a gap, so the book goes stale. A max-occupancy counter lets me validate the sizing on real traffic.

> **Sizing reasoning:** the only thing that stalls Block 2 is an L1 miss (DDR round trip, a few hundred ns ≈ 50–100 cycles). At one message every ~2.6 cycles worst case, a single miss backs up ~40 events. 512 entries absorbs a burst of ~10 consecutive misses with margin. A 64-entry FIFO (1 BRAM) would overflow on the second back-to-back miss; 4096 entries would just cost BRAM for a scenario that already means the book is stale.

### 3.10 Parser latency

~2–4 cycles after the last byte of a message arrives (assembly register → decode → locate lookup, overlapped) ≈ **13–26 ns**.

---

## 4. Block 2 — Order memory (hash table, L1 + L2)

### 4.1 Function

Store every live order as `order_ID → {side, price, qty}`, because Execute / Cancel / Delete / Replace carry only the ID. Output to the price book is a signed quantity delta at a price:

| Event | Output(s) to price book |
|---|---|
| ADD | `+qty` at the new price |
| EXEC / CANCEL | `−qty_removed` at the stored price |
| DELETE | `−remaining_qty` at the stored price |
| REPLACE | `−remaining_qty` at the old price, then `+new_qty` at the new price (same side) |

### 4.2 Entry format

| Field | Bits |
|---|---|
| valid | 1 |
| order_id (full, used as the tag) | 64 |
| side | 1 |
| price | 32 |
| qty | 32 |
| book_idx | 3 |
| reserved | 11 |
| **Stored width** | **144 = 2 × 72-bit URAM** |

The payload (side + price + qty) is only 65 bits; the full 64-bit ID is stored so a hit can be *verified*, exactly like a cache tag. Storing only the ID bits not implied by the slot address (50 bits instead of 64) would save nothing here because the entry would still round up to 144 bits — but it would matter if I were squeezing into a narrower external memory.

### 4.3 Hash function — XOR folding

Slots = 2^14 = 16,384, so I need a 14-bit hash. I XOR-fold the 64-bit ID into 14-bit chunks:

```systemverilog
function automatic logic [13:0] hash14(input logic [63:0] id);
  return id[13:0] ^ id[27:14] ^ id[41:28] ^ id[55:42] ^ {6'b0, id[63:56]};
endfunction
```

- Purely combinational: 4 XOR levels per output bit → one cycle, registered.
- Every ID bit affects the address.
- **Pigeonhole:** 2^64 / 2^14 = 2^50 possible IDs map to every slot, so collisions are *guaranteed to be possible*. What matters is collisions among **live** orders, which is a statistical question (4.6).
- Colliding IDs for tests are trivial to craft: `id` and `id ^ (x << 14) ^ x` collide for any 14-bit `x`.

**Alternative — H3 hashing** (each output bit = XOR of a random subset of input bits). I'd switch to it if real IDs showed patterns that XOR-folding maps badly. ITCH order IDs generally increase through the day, which XOR-folding spreads well, but it has to be checked by histogramming slot occupancy on sample data (listed in 11).

### 4.4 Why 2^14 slots

- A URAM is **4096 deep = 12 address bits**. The hash must be ≥ 12 bits or URAM rows are wasted: a 10-bit hash would leave 75% of every block unused.
- 2^14 slots = **4 URAM depths cascaded**: the low 12 bits address within a block, the upper 2 select the cascaded block. Vivado infers the cascade chain.
- This uses **32 of the 64 URAMs** — half the chip's URAM, deliberately leaving room to double.

> **What if 2^13 or 2^15?** 2^13 would use 16 URAMs and halve capacity (λ doubles for the same order count). 2^15 would use all 64 URAMs, leaving nothing for growth and making the whole design hostage to the cascade timing. 2^14 at 4 ways is the balanced point; see the ways trade-off below for the equivalent-URAM alternative.

### 4.5 Ways — 4-way set-associative

Each way is a **separate 144-bit-wide memory**; all four get the **same address** and are read **in parallel in one access**:

```
way 0 → URAM pair (2 × 72 bits) × 4 deep-cascaded = 8 URAMs
way 1 → 8 URAMs
way 2 → 8 URAMs
way 3 → 8 URAMs
total  = 32 URAMs, 16,384 slots × 4 ways = 65,536 entries
```

```systemverilog
(* ram_style = "ultra" *) entry_t way0 [0:16383];
(* ram_style = "ultra" *) entry_t way1 [0:16383];
(* ram_style = "ultra" *) entry_t way2 [0:16383];
(* ram_style = "ultra" *) entry_t way3 [0:16383];
// registered (synchronous) reads, same address to all four; separate write enable per way
```

- Separate memories per way give a **per-way write enable**: an update writes only the modified way, not a 576-bit bucket.
- URAM port width is fixed at 72 bits; wider words are built by placing blocks side by side (same latency, more blocks + routing).
- I verify in the utilisation report that URAM (not BRAM or LUTs) was actually inferred.

**Trade-off of more ways:**

| More ways → | Effect |
|---|---|
| Overflow probability | ↓ sharply, at the same total capacity |
| Comparators + output mux | ↑ N × 64-bit compares + N:1 mux → LUTs, harder timing |
| Timing | may need +1 pipeline stage → +1 cycle latency *and* a longer hazard window |
| Memory word width | ↑ more blocks per access, routing congestion |
| On external memory | wider bucket = more bus beats on a fixed-width DDR/QDR bus → more latency |

### 4.6 Overflow probability — the math behind the sizing

Assuming a uniform hash, the number of live orders in one slot is Binomial(M, 1/B), which for large M and small 1/B is **Poisson(λ)** with **λ = M / B** (M = live orders, B = 16,384 slots). A slot overflows when it holds more orders than it has ways:

**P(overflow) = P(k > N) = 1 − Σ_{k=0}^{N} e^(−λ) λ^k / k!**, with N = 4.

| Live orders M | λ | P(slot > 4) | Expected overflowing slots (× 16,384) |
|---|---|---|---|
| 4,096 | 0.25 | ≈ 0.0006 % | ~0 |
| 8,192 | 0.5 | ≈ 0.017 % | ~3 |
| 16,384 | 1 | ≈ 0.37 % | ~60 |
| 32,768 | 2 | ≈ 5.3 % | ~860 |
| 65,536 | 4 | ≈ 37 % | ~6,100 |

Worked example for λ = 1: e^−1 × (1 + 1 + 1/2 + 1/6 + 1/24) = 0.3679 × 2.7083 = 0.9963 → P(overflow) = 0.37 %.

**Design point: keep λ ≤ 0.5** (≤ ~8K live orders across the 8 tracked books in L1). For 8 liquid stocks that's comfortable. Above that the LRU/L2 mechanism keeps the design *correct*; only the latency of evicted orders suffers. Crucially, in this design an overflow is **not data loss** — it causes an LRU eviction to L2.

**What if I'd chosen a different associativity for the same 32 URAMs?**

| Configuration | Slots | λ at 8K orders | P(overflow) | Slots overflowing | Comparators |
|---|---|---|---|---|---|
| 2-way, 2^15 slots | 32,768 | 0.25 | 1 − e^−0.25(1 + 0.25 + 0.031) ≈ **0.2 %** | ~70 | 2 × 64-bit |
| **4-way, 2^14 slots** | 16,384 | 0.5 | ≈ **0.017 %** | ~3 | 4 × 64-bit |
| 8-way, 2^13 slots | 8,192 | 1.0 | ≈ **0.001 %** | ~0 | 8 × 64-bit |

Same capacity, same URAM count, wildly different overflow rates — associativity is far more effective than slot count at the same memory budget. 8-way would be ~17× better still, at the cost of twice the comparators, a 1152-bit read word, and probably one more pipeline stage. 4-way is the point where overflow is already negligible and the compare/mux logic trivially closes timing at 156 MHz; 8 ways (using all 64 URAMs at 2^14) is the obvious upgrade if I ever need λ ≈ 1.

### 4.7 Replacement policy — pseudo-LRU

| Scheme | Bits/slot (4-way) | Notes |
|---|---|---|
| True LRU, 2-bit age per way | 8 | ages are a permutation of 0..3; on access, younger ways +1, accessed → 0; victim = age 3 |
| True LRU, minimal encoding | 5 | 4! = 24 orderings ≤ 32; decoding costs logic |
| **Pseudo-LRU tree** | **3** | ✅ chosen — approximate, very cheap |

```
        b0
      /    \
    b1      b2
   /  \    /  \
  w0  w1  w2  w3
```

- Bits point toward the side to evict (0 = left, 1 = right).
- **Victim:** follow bits from the root. `b0 = 1 → right → b2 = 0 → w2`.
- **On access to a way:** set the bits on its path to point *away* from it (access w0 → `b0 = 1, b1 = 1`).
- Example: `000` → victim w0; access w0 → `110` → victim w2; access w2 → `011` → victim w1.
- **Victim on Add:** first free way (`valid = 0`) if any, else the PLRU victim.

**Storage:** 16,384 × 3 bits = 48 Kbit → **2 BRAM36**, in a separate memory indexed by the same slot address. The bits change on *every* access (including reads), and I don't want every lookup to become a 144-bit URAM write. The alternative — storing them in the entry's spare padding bits — would turn every hit into a URAM write and halve effective port bandwidth.

> **Why not true LRU?** 8 bits × 16K = 128 Kbit (4 BRAM36) plus permutation-update logic, for marginal benefit: with λ ≤ 0.5 almost no slot ever holds 4 live orders, so the quality of the eviction choice barely matters. The 3-bit tree is the standard CPU-cache answer for exactly this reason.

### 4.8 Two-level hierarchy — L1 cache of L2, inclusive, write-through

| Level | Where | Size | Latency | Contents |
|---|---|---|---|---|
| L1 | URAM (on-chip) | 65,536 entries | fixed, ~5–7 cycles | hot subset |
| L2 | PS DDR4 via AXI HP port (`S_AXI_HP0_FPD`, 128-bit) | 2^20 slots × 8 ways = 8M entries | hundreds of ns, **variable** | **every** live order |

- **Inclusive:** every order in L1 is also in L2.
- **Write-through:** every ADD/EXEC/CANCEL/DELETE/REPLACE updates L1 (if present) *and* posts a write to L2 through a write FIFO. The fast path never waits for DDR.
- **Eviction is free:** L2 is always up to date, so an evicted L1 entry is simply overwritten — no dirty bits, no write-back.
- **Delete always works:** an L1 miss falls through to L2.

**L2 layout:** XOR-fold to 20 bits (2^20 slots), 8 ways per slot. Entry = 32 bytes (valid, book_idx, side, id, price, qty, padding) → slot = 256 bytes = 4 × 64-byte bursts. Footprint = 2^20 × 256 B = **256 MB** of the 4 GB, reserved in the Linux device tree (`reserved-memory`) so the OS never touches it. λ for L2 is tiny (e.g. 16K orders / 1M slots ≈ 0.016) → L2 overflow practically never happens; if it does → error flag + book stale.

**L2 controller:** **one in-order command queue** for both write-throughs and miss reads. Because commands are processed strictly in order, a read issued after a write always observes that write — no extra consistency logic. Each write-through is a read-modify-write of the L2 slot (find the matching or a free way), done in the background. Cost: a miss may wait behind queued writes → longer miss latency, which I measure.

**Bandwidth check:** each message costs ~256 B read + 32 B write in DDR ≈ 288 B. At 3 M messages/s that's ~0.86 GB/s, well under what a 128-bit HP port delivers; I validate under replayed peak bursts.

> **Why write-through rather than write-back?** Write-back halves DDR traffic (only evictions write) but needs dirty bits, a write-back on every eviction (so eviction is no longer free), and the L2 can be stale — a miss after an eviction must wait for the write-back to land. Write-through trades DDR bandwidth, which I have in abundance, for simplicity and a backing store that is *always* consistent. On a design where DDR bandwidth was the constraint I'd flip this.

### 4.9 Pipeline (156.25 MHz)

| Stage | All ops | ADD | EXEC / CANCEL | DELETE | REPLACE (2 micro-ops) |
|---|---|---|---|---|---|
| S0 | hash(ID) → slot; register | | | | µop1 = DELETE(old ID) |
| S1–S3 | read 4 ways (URAM, ~2–3 cycles incl. cascade + output registers) + PLRU bits (BRAM) | | | | |
| S4 | hazard check / forwarding; 4 parallel ID compares + valid → hit way | must miss (else duplicate-add error) | must hit | must hit | must hit |
| S5 | compute new entry, victim, new PLRU | free way or PLRU victim; build entry | qty −= removed; if 0 → valid = 0 | valid = 0 | µop1: as DELETE, capture side; µop2 = ADD(new ID, captured side, new price, new qty) |
| S6 | write the one modified way + PLRU; emit price-book delta; push write-through to L2 queue | | | | |

- Latency ≈ 7 cycles ≈ **45 ns** on a hit.
- Throughput ≤ 1 op/cycle. REPLACE takes 2 slots, which fits because a `U` message occupies ≥ 4 input beats on the wire.
- **L1 miss** (EXEC/CANCEL/DELETE/REPLACE not found):
  - v1 **blocking**: stall the pipeline, read L2 via the queue, insert the order into L1 (evicting the PLRU victim), complete the op. The input FIFO absorbs the stall.
  - v2 **non-blocking**: an MSHR-style miss table (~8 entries); later events for the same slot or ID wait, unrelated ones proceed.
- **Not in L2 either** → unknown order (e.g. added before capture started) → drop and count.

### 4.10 Read-modify-write hazards and forwarding

An op reads its slot at S1 but writes at S6. A younger op to the **same slot** reading in between sees stale data — two back-to-back executions on the same order, or two adds picking the same free way, would corrupt the table.

**Decision: forwarding at slot granularity.** The in-flight ops in S2…S6 live in a 5-deep shift register `{slot, updated 4-way bucket, updated PLRU}`. At S4, if the incoming slot matches an older in-flight slot, the *youngest* matching forwarded bucket replaces the URAM data. Cost: ~5 × 14-bit comparators + a mux. Forwarding on the slot (not the ID) also handles the "two adds choose the same free way" case for free.

*Simpler alternative:* stall the younger op until the older has written. Acceptable in practice because message spacing on the wire is ≥ ~2.6 cycles on average, but it makes throughput data-dependent — forwarding keeps it fixed.

Every pipeline stage added for timing lengthens this hazard window by one — which is the real cost of chasing a faster clock.

### 4.11 Memory sizing reference

- Per way: 16,384 × 144 bits = 288 KB (8 URAMs). Four ways = 1.15 MB on-chip.
- The large-FPGA design I'd build with unlimited resources — 2^20 slots × 130 bits ≈ 16 MB *per way* — doesn't fit anything in a home budget; hence 2^14 slots on-chip and 2^20 in DDR.

---

## 5. Block 3 — Price book and best-price tracking

### 5.1 Function

Per stock and per side, keep the **total quantity at each price level**, and output best bid / best ask whenever either changes.

How the book moves:
- **Add** creates or grows a level. The best bid only changes if the new buy price is *higher*; the best ask only if the new sell price is *lower*.
- **Execute / Cancel** shrinks a level.
- **Level reaches 0** → it disappears. If it was the best, the best moves to the next level — the hard case, because it requires a *search*.

### 5.2 Price representation

ITCH prices are 32-bit integers in 1/10000 $ (50.12 → 501200). With a $0.01 tick for stocks ≥ $1, valid prices are multiples of 100.

Level index: `tick = (price − base_price) / 100`. Division is slow in hardware, so I **multiply by 83,887 and shift right by 23** (one DSP):

- 2^23 / 100 = 83,886.08 → rounded up to 83,887.
- Relative error = 0.92 / 83,886 ≈ 1.1 × 10^−5.
- For `diff = 100k`, the product is `k × (1 + 1.1e−5)`; the floor stays `k` as long as `k × 1.1e−5 < 1`, i.e. **k < ~91,000 ticks** ($910 of range) — far more than the 1024-tick window needs.
- I check `diff % 100 == 0` (sub-penny price → count, don't book). Only stocks ≥ $1 are supported in v1.
- Output converts back: `price = base_price + tick × 100`.

### 5.3 Level storage — dense price window

- A **dense window of 1024 ticks** per book and side (±512 around base = **±$5.12**).
- Level entry: 32-bit aggregate qty (optionally a 16-bit order count).
- Address = `{book_idx[2:0], side, tick[9:0]}` = 14 bits → 16,384 × 32 bits = 512 Kbit → **16 BRAM36**. BRAM fits better than URAM here: the array is small and BRAM's width/depth are configurable.
- Port A: read-modify-write of the updated level. Port B: read the qty of the (new) best level.
- **Base price** is set by the PS per book (previous close, or first add of the day).
- **Out-of-window prices**: the order is still stored in Block 2 but not in the level array; counted as `out_of_window`. They only affect the BBO if the market moves more than $5 intraday, which is rare for the stocks I track and flagged when it happens.

> **Why dense rather than a sorted structure?** A sorted list/tree gives O(log n) insertion with variable latency and pointer chasing — exactly what I'm avoiding. A dense array makes every level update a single fixed-latency RMW and turns "find the best" into a bit-search problem (5.4).
>
> **What if the window were different?** 2048 ticks (±$10.24) doubles BRAM to 32 and bitmap FFs to 32K, and the priority encoder grows a level (64 groups of 32, or 32 groups of 64) — still fine. 256 ticks (±$1.28) would be too tight for a volatile stock. A circular window (`tick mod 1024`) with recentering is the "proper" fix but requires rebuilding the levels from order memory on every recenter — not v1.

### 5.4 Best-price tracking — bitmap + hierarchical priority encoder

- **Occupancy bitmap:** 1 bit per level (1 = qty > 0) in flip-flops: 8 books × 2 sides × 1024 = **16K FFs**.
- A flat 1024-bit priority encoder won't close timing in one cycle, so it's **hierarchical**:
  1. A 32-bit summary vector per book/side (bit *g* = OR of group *g*'s 32 bits), maintained incrementally on every level update.
  2. Priority-encode the summary → group index (highest set bit for bids, lowest for asks).
  3. Select that 32-bit group, priority-encode → bit index. `tick = {group, bit}`.
- **v1:** run the encoder on every update — fixed ~3-cycle latency, uniform behaviour, easy to verify.
- **v2 (hybrid):** keep `best_tick` registers; on ADD just compare (1 cycle); run the encoder only when the best level empties.
- **Sanity check:** best bid ≥ best ask (crossed book) → flag. Indicates a bug or a missed message.

> **Why bitmap in FFs rather than BRAM?** The encoder needs the whole 32-bit group and the summary vector combinationally in the same cycle; a BRAM read would add 1–2 cycles and the summary would need a second port. 16K flip-flops is ~6% of the chip's FFs — affordable.

### 5.5 Pipeline (156.25 MHz)

| Stage | Action |
|---|---|
| P0–P1 | price → tick (DSP multiply/shift), window check |
| P2–P3 | read level qty (BRAM port A) |
| P4 | forwarding (same `{book, side, tick}` in flight); `new_qty = qty + delta`; detect 0 ↔ non-zero transition |
| P5 | write level qty; update bitmap bit + summary bit |
| P6–P8 | hierarchical priority encoder → best tick; read best level qty (port B) |
| P9 | if BBO changed: tick → price, push BBO update |

Latency ≈ 9–10 cycles ≈ **60 ns**. Same RMW hazard as Block 2 → forwarding on `{book, side, tick}`.

---

## 6. Control plane and software

### 6.1 AXI-Lite register map

| Group | Registers |
|---|---|
| Control | enable, soft reset, clear stale flag, invalidate book(s) |
| Network | dst MAC, multicast IP, UDP port, VLAN enable |
| MoldUDP64 | session ID (10 B), expected_seq init, current expected_seq |
| Symbols | 8 × 8-byte symbols + enable bits; direct locate-table write port |
| Price book | base price per book |
| Counters | packets rx / dropped / bad FCS; messages per type; gaps, duplicates; FIFO overflow + max occupancy; L1 hits / misses / evictions; L2 overflow; unknown orders; duplicate adds; out-of-window; crossed book |
| Readback | current BBO per book |

### 6.2 PS software

Linux on the A53 cores: configure registers, subscribe symbols, receive the BBO stream over DMA, log, handle gaps/recovery, read counters. The 256 MB L2 region is reserved in the device tree.

---

## 7. Resource budget (XCK26: 64 URAM, 144 BRAM36)

| Use | Resource |
|---|---|
| L1 order table (4 ways × 2^14 slots × 144 bits) | 32 URAM (50 %) |
| PLRU bits (16,384 × 3) | 2 BRAM36 |
| Stock locate table (65,536 × 4) | ~8 BRAM36 |
| Price levels (16,384 × 32) | 16 BRAM36 |
| Input event FIFO, L2 command/write FIFOs, BBO FIFO | ~6–10 BRAM36 |
| Occupancy bitmaps + summaries | ~16K FFs |
| Tick conversion | 1 DSP |
| **Total BRAM** | **~35 / 144** |

Plenty of headroom to scale to 8 ways (all 64 URAMs), more books, or a wider price window.

---

## 8. Latency budget (6.4 ns/cycle)

| Segment | Estimate |
|---|---|
| PHY / PCS / MAC RX | IP-dependent, tens of ns |
| Message arrival | fixed by the wire (message length / position) |
| Parser after last byte | 2–4 cycles ≈ 13–26 ns |
| Order memory (L1 hit) | ~7 cycles ≈ 45 ns |
| Price book + best | ~9–10 cycles ≈ 60 ns |
| **Last byte → BBO update** | **≈ 20 cycles ≈ 130 ns** (L1 hit) |
| L1 miss | + one DDR round trip (hundreds of ns, variable) |

Measurement method: hardware timestamp at MAC RX `tvalid` of the first beat and at BBO output; histogram min / mean / max. **Jitter matters as much as the mean** — a deterministic 130 ns is worth more than a 100 ns average with a 2 µs tail.

For comparison, a tuned software order book on a kernel-bypass NIC typically lands in the low single-digit microseconds with a long tail; the FPGA path is ~10–20× faster and, more importantly, flat.

---

## 9. Verification

- **Test data:** Nasdaq's public sample TotalView-ITCH 5.0 files (binary, length-prefixed messages).
- **Python packetiser:** split messages into MoldUDP64 packets (configurable messages/packet, sequence numbers), wrap in UDP/IPv4/Ethernet with scapy → `.pcap`.
- **Python golden model:** dict `order_id → (side, price, qty)` + per-stock level maps → expected BBO stream.
- **Simulation:** cocotb + Verilator (or Vivado xsim) driving AXI-Stream from the pcap; hardware BBO output compared to the golden model.
- **Directed tests:**
  - Messages at every lane offset; VLAN / no VLAN; a message spanning 6 beats; two messages in one beat.
  - Back-to-back ops on the same order and the same slot (hazards).
  - Crafted colliding IDs (`id ^ (x<<14) ^ x`) to overfill a slot → evictions → L2 hits.
  - Gaps, duplicates, partial duplicates, heartbeats, end of session, bad FCS, truncated packets.
  - Replace (side inheritance), execute-to-zero at the best level (next-best search), out-of-window prices.
- **On hardware:** PC with a 10G NIC + SFP+ DAC → `tcpreplay` the pcap at line rate; BBO DMA output compared to the golden model; counters checked; ILA for debug.

---

## 10. Build milestones

1. Parser in simulation (assembler + decoder) against the golden model.
2. Order memory L1 only (no eviction — error on overflow), with forwarding.
3. Price book + priority encoder → full BBO in simulation.
4. PLRU + L2 in DDR (simulated first with an AXI memory model).
5. Bring-up on the KR260: MAC/PCS, loopback, then the `tcpreplay` feed.
6. Latency measurement + counters; then extensions.

---

## 11. What I'd do differently / extensions

- **Non-blocking misses** (MSHR table) so a single L2 lookup doesn't stall unrelated stocks.
- **Hybrid best tracking** (compare-only on adds) to shave ~2 cycles off the common case.
- **Faster core clock** behind an async FIFO once the single-domain design is proven.
- **A/B line arbitration** on real hardware with a second port.
- **Circular price window with recentering** for stocks that move more than $5 in a day.
- **8-way L1** using all 64 URAMs if I want to track more or busier symbols (λ ≈ 1 at 0.001 % overflow).
- **Things to confirm against the hardware:** KR260 SFP+ wiring to PL GTH transceivers and Ethernet IP licensing; ITCH field offsets against the official spec; real order-ID distribution through XOR-fold-14 (slot-occupancy histogram); URAM read latency with the 4-deep cascade from the timing report (sets the final S1–S3 length); DDR latency/bandwidth through the HP port under load.


