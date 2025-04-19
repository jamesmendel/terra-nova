Clone Firmware repo:
```bash
git clone --filter=blob:none --no-checkout https://github.com/jamesmendel/terra-nova.git
cd terra-nova
git sparse-checkout init --cone
git sparse-checkout set Firmware
git checkout main
```



Install PlatformIO

Initialize the project:
```bash
cd Firmware/Terra
pio init --ide vscode
```