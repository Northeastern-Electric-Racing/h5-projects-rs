# Cerberus 3

### Instructions for Probe-rs remote flashing
Install tools: ```cargo install probe-rs-tools --features remote --force --locked``` <br>
Stop probe-rs server (on root@192.168.100.12): ```/etc/init.d/S*probe-rs stop```, then run ```probe-rs serve```

Flash remotely via (in cerberus-3 dir): ```cargo run -p cerberus-3 -- --host ws://192.168.100.12:3000 --token ner```