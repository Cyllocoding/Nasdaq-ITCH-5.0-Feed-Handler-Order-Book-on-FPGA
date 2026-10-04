# Nasdaq-ITCH-5.0-Feed-Handler-Order-Book-on-FPGA
deterministic market-data pipeline on a Zynq UltraScale+ that ingests a Nasdaq TotalView-ITCH 5.0 feed over 10G Ethernet (MoldUDP64/UDP multicast), tracks every live order for eight subscribed stocks, and emits best-bid/best-ask updates in a fixed ~130 ns from last byte on the wire
