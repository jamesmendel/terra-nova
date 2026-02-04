Import("env")
import os

build_dir = env.subst("$BUILD_DIR")
project_dir = env.subst("$PROJECT_DIR")

version = env.GetProjectOption("custom_version", "0.0.0")
output_bin = os.path.join(build_dir, f"terra_{version}.bin")

bootloader = os.path.join(build_dir, "bootloader.bin")
partitions = os.path.join(build_dir, "partitions.bin")
firmware = os.path.join(build_dir, "firmware.bin")

framework_pkg = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
boot_app0 = os.path.join(framework_pkg, "tools/partitions/boot_app0.bin")

tool_pkg = env.PioPlatform().get_package_dir("tool-esptoolpy")
esptool = os.path.join(tool_pkg, "esptool.py")

python = env.subst("$PYTHONEXE")

def merge_action(source, target, env):
    cmd = f'"{python}" "{esptool}" --chip esp32s3 merge_bin -o "{output_bin}" ' \
          f'--flash_mode dio --flash_freq 80m --flash_size 4MB ' \
          f'0x0000 "{bootloader}" ' \
          f'0x8000 "{partitions}" ' \
          f'0xE000 "{boot_app0}" ' \
          f'0x10000 "{firmware}"'

    # print(f"command: {cmd}")
    # env.Execute(cmd)
    
    return env.Execute(cmd)

env.AddCustomTarget(
    name="merge_firmware",
    dependencies=["buildprog"],
    actions=[
        env.VerboseAction(
            merge_action,
            "Merging firmware into single binary"
        )
    ],
    title="Merge firmware",
    description="Create merged image at 0x0000 for dist"
)