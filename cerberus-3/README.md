# Cerberus 3

## Instructions for Can remote flashing
Clone the firmware-rs repository, build the can-flash-cli, and then follow the instructions in the README for that folder.

```can flash --host "http://192.168.100.12:8083"```

### Instructions for Probe-rs remote flashing
Install tools: ```cargo install probe-rs-tools --features remote --force --locked``` <br>
Stop probe-rs server (on root@192.168.100.12): ```/etc/init.d/S*probe-rs stop```, then run ```probe-rs serve```

Flash remotely via: ```cargo run --release -p cerberus-3 -- --host ws://192.168.100.12:3000 --token ner```

Reconnect to the running process via: ```probe-rs attach --no-catch-reset --chip STM32H563ZITx --host ws://192.168.100.12:3000 --token ner target/thumbv8m.main-none-eabihf/release/cerberus-3```