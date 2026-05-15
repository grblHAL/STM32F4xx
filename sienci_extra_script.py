import datetime
Import("env")

# 1. Generate the realtime date
# Format: 20260115-1430
build_date = datetime.datetime.now().strftime("%Y%m%d-%H%M")

# 2. Get the variables from platformio.ini
custom_ver = env.GetProjectOption("custom_prog_version")     # SLB_EXT

# 3. Construct the new program name
# We removed the extra "%s" and the extra "." since you removed the driver version
# Result Example: SLB_EXT_20260115-1430
new_prog_name = "%s_%s" % (custom_ver, build_date)

# 4. Apply the new name to the environment
env.Replace(PROGNAME=new_prog_name)
env.Replace(custom_board_name=new_prog_name)

# 5. Custom HEX from ELF
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "ihex", "-R", ".eeprom",
        '"$BUILD_DIR/${PROGNAME}.elf"', '"$BUILD_DIR/${PROGNAME}.hex"'
    ]), "Building $BUILD_DIR/${PROGNAME}.hex")
)
