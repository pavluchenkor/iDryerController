Import("env")
import os

def after_build(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    eep_file = os.path.join(build_dir, "firmware.eep")
    if os.path.isfile(eep_file):
        size = os.path.getsize(eep_file)
        percent = (size / 1024) * 100  # 1024 байта EEPROM для ATmega328P
        print("\n===================================")
        print(f" EEPROM file size: {size} bytes ({percent:.2f}% used)")
        print("===================================")

env.AddPostAction("$BUILD_DIR/firmware.eep", after_build)
