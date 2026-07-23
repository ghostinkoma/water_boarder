#!/bin/sh
# Regenerate the 3.3V firmware from the 5V source (single source of truth).
# The only differences are the build-title banner and the TEMP_HIGH_RANGE line.
sed -e 's/BUILD: 5V (high-range/BUILD: 3.3V (low-range/' \
    -e 's/#define TEMP_HIGH_RANGE  1/#define TEMP_HIGH_RANGE  0/' \
    i2c_wb_5V.c > i2c_wb_3V3.c
echo "i2c_wb_3V3.c regenerated from i2c_wb_5V.c"
diff i2c_wb_5V.c i2c_wb_3V3.c   # should show only the title + TEMP_HIGH_RANGE lines
