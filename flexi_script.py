Import("env")

# Custom HEX from ELF
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "ihex", "-R", ".eeprom", 
        '"$BUILD_DIR/${PROGNAME}.elf"', '"$BUILD_DIR/${PROGNAME}.hex"'
    ]), "Building $BUILD_DIR/${PROGNAME}.hex")
)

# Custom UF2 from BIN
def after_build(source, target, env): 
    print("Building $BUILD_DIR/${PROGNAME}.uf2")
    env.Execute("python uf2conv.py -c -b 0x08010000 -f 0x57755a57 $BUILD_DIR/firmware.bin --output $BUILD_DIR/${PROGNAME}.uf2")

env.AddPostAction("buildprog", after_build)