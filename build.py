import os
import subprocess
from pathlib import Path

ICC = "icc8051"
LD = "xlink"

TARGET = "CC2541-HID"
SRC_DIR = "src"
BUILD_DIR = "build"
BIN_DIR = "bin"

TOOLKIT_DIR = "C:/PROGRA~2/IARSYS~1/EMBEDD~1.3/8051"
RUNTIME_TYPE = "clib"
RUNTIME_LIB = "cl-pli-blxd-1e16x01.r51"

BLE_SDK_DIR = "C:/PROGRA~2/TEXASI~1/BLE-CC254x-1.5.0.16"
LD_CONFIG = "ti_51ew_cc2540b.xcl"

BLE_LIBS = [
    f"{BLE_SDK_DIR}/Projects/ble/Libraries/CC2541DB/bin/CC2541_BLE_peri.lib",
    f"{BLE_SDK_DIR}/Projects/ble/Libraries/Common/bin/CC254x_BLE_HCI_TL_None.lib"
]

# for any library .c files that cannot be found via header pattern
LIB_SRC_DIRS = f'''\
{BLE_SDK_DIR}/Projects/ble/Profiles/GATT
{BLE_SDK_DIR}/Components/osal/common
'''

# for any library.c files that cannot be found via header pattern,
# and it is a smaller headache to include the file rather than the dir
LIB_SRC_FILES = [
    f"{BLE_SDK_DIR}/Components/hal/target/CC2540EB/hal_startup.c"
]

INCLUDE_DIRS = f'''\
{TOOLKIT_DIR}/inc
{TOOLKIT_DIR}/inc/{RUNTIME_TYPE}
{BLE_SDK_DIR}/Projects/ble/include
{BLE_SDK_DIR}/Components/hal/include
{BLE_SDK_DIR}/Components/hal/common
{BLE_SDK_DIR}/Components/hal/target/CC2540EB
{BLE_SDK_DIR}/Components/osal/include
{BLE_SDK_DIR}/Components/osal/common
{BLE_SDK_DIR}/Components/osal/mcu/cc2540
{BLE_SDK_DIR}/Components/services/saddr
{BLE_SDK_DIR}/Components/ble/include
{BLE_SDK_DIR}/Components/ble/hci
{BLE_SDK_DIR}/Components/ble/host
{BLE_SDK_DIR}/Projects/ble/common/cc2540
{BLE_SDK_DIR}/Components/ble/controller/CC254x/include
{BLE_SDK_DIR}/Projects/ble/Profiles/Roles
{BLE_SDK_DIR}/Projects/ble/Profiles/HIDDevKbM
{BLE_SDK_DIR}/Projects/ble/Profiles/HidDev
{BLE_SDK_DIR}/Projects/ble/Profiles/DevInfo
{BLE_SDK_DIR}/Projects/ble/Profiles/Batt
{BLE_SDK_DIR}/Projects/ble/Profiles/ScanParam
{BLE_SDK_DIR}/Projects/ble/Profiles/Roles/CC254x
{BLE_SDK_DIR}/Projects/ble/Profiles/HIDDevKbM/CC254x
{BLE_SDK_DIR}/Projects/ble/Profiles/HidDev/CC254x
{BLE_SDK_DIR}/Projects/ble/Profiles/Batt/CC254x
{BLE_SDK_DIR}/Projects/ble/Profiles/ScanParam/CC254x
{BLE_SDK_DIR}/Projects/ble/Profiles/SimpleProfile
{BLE_SDK_DIR}/Projects/ble/Profiles/SimpleProfile/CC254x
{BLE_SDK_DIR}/Projects/ble/Profiles/Keys
{BLE_SDK_DIR}/Projects/ble/Profiles/Keys/CC254x
'''

ICCFLAGS = f'''\
-Ohz
--debug
--core=plain
--code_model=banked
--data_model=large
-e
--calling_convention=xdata_reentrant
-f{BLE_SDK_DIR}/Projects/ble/config/buildComponents.cfg
-fbuildConfig.cfg
--nr_virtual_regs=16
--place_constants=data_rom
--warnings_affect_exit_code'''

SYMBOLS = '''\
INT_HEAP_LEN=3072
HALNODEBUG
OSAL_CBTIMER_NUM_TASKS=1
HAL_AES_DMA=TRUE
HAL_DMA=TRUE
POWER_SAVING
xPLUS_BROADCASTER'''

#GAP_CHAR_CFG_MAX=6
#HAL_UART=TRUE
#HAL_UART_ISR=2
#HAL_UART_DMA=FALSE'''

# changed PDATA_STACK_SIZE from 0x80 to 0x00
LDFLAGS = f'''\
-Ointel-extended={BIN_DIR}/{TARGET}.hex
-D_NR_OF_BANKS=0x07
-D_CODEBANK_START=0x8000
-D_CODEBANK_END=0xFFFF
-D_NR_OF_VIRTUAL_REGISTERS=10
-D?PBANK=0x93
-D?CBANK=0x9F
-D?CBANK_MASK=0xFF
-D_IDATA_STACK_SIZE=0xC0
-D_PDATA_STACK_SIZE=0x00
-D_XDATA_STACK_SIZE=0x280
-D_XDATA_HEAP_SIZE=0xFF
-D_FAR_HEAP_SIZE=0xFFF
-D_HUGE_HEAP_SIZE=0xFFF
-D_FAR22_HEAP_SIZE=0xFFF
-D_EXTENDED_STACK_START=0
-D_EXTENDED_STACK_SIZE=0
-e_large_write=_formatted_write
-e_large_read=_formatted_read
-e?BCALL_FF=?BCALL
-e?BRET_FF=?BRET
-e?BDISPATCH_FF=?BDISPATCH
-s __program_start
-rt
{TOOLKIT_DIR}/lib/{RUNTIME_TYPE}/{RUNTIME_LIB}
-f {BLE_SDK_DIR}/Projects/ble/common/cc2540/{LD_CONFIG}
-l {BUILD_DIR}/{TARGET}.map
-xmsn
'''

INCLUDE_DIRS = INCLUDE_DIRS.splitlines()
INCLUDE_DIRS.append(SRC_DIR)

ICCFLAGS = ICCFLAGS.splitlines()
LDFLAGS = LDFLAGS.splitlines()

for directory in INCLUDE_DIRS:
    ICCFLAGS.append(f"-I{directory}")

for symbol in SYMBOLS.splitlines():
    ICCFLAGS.append(f"-D{symbol}")

compile_command = [ ICC ] + ICCFLAGS
link_command = [ LD ] + LDFLAGS

def get_sources():
    '''
    Generates a list of .c files in the source directory and
    library source directory.
    '''
    sources = []
    for file in os.listdir(SRC_DIR):
        if file.endswith(".c"):
            sources.append(os.path.join(SRC_DIR, file))

    for d in LIB_SRC_DIRS.splitlines():
        for file in os.listdir(d):
            if file.endswith(".c"):
                sources.append(os.path.join(d, file))

    return sources + LIB_SRC_FILES

def clean():
    '''
    Deletes .r51, .map, .d, .hex, and .bin files.
    '''
    os.system(f"rm {BUILD_DIR}/*.r51 -f")
    os.system(f"rm {BUILD_DIR}/*.map -f")
    os.system(f"rm {BUILD_DIR}/*.d -f")
    os.system(f"rm {BIN_DIR}/*.hex -f")
    os.system(f"rm {BIN_DIR}/*.bin -f")

def get_file_name(file):
    return os.path.splitext(os.path.basename(file))[0]

not_found_headers = set()

def get_header_source(header_file):
    global not_found_headers
    if header_file in not_found_headers:
        return None

    file_name = get_file_name(header_file)

    for d in [ SRC_DIR ] + INCLUDE_DIRS:
        for file in os.listdir(d):
            if file == file_name + ".c":
                return os.path.join(d, file_name + ".c")

    print(f"Could not find source file for {header_file}")
    not_found_headers.add(header_file)

    return None

def get_object_files(source_files):
    '''
    Given a list of source files, returns a list of the corresponding
    object files.
    '''
    objects = []
    for file in marked_sources:
        file_name = get_file_name(file)
        objects.append(f"{os.path.join(BUILD_DIR, file_name)}.r51")

    return objects

PREAMBLE_END = "dition 10.40\n "

def build_file(command_stem, source_file):
    file_name = get_file_name(source_file)
    dependency_file = f"{os.path.join(BUILD_DIR, file_name)}.d"
    command = command_stem + [ 
       f"-o{os.path.join(BUILD_DIR, file_name)}.r51", 
       source_file,
       "--dependencies", 
       dependency_file ]

    # TODO: skip building file if the .r51 object file is more recent than the source

    print(f"Compiling {source_file}")
    result = subprocess.run(command, text=True, capture_output=True)
    
    if PREAMBLE_END in result.stdout:
        output = result.stdout.split(PREAMBLE_END)[1] # remove preamble
    else:
        output = result.stdout        
        
    print(output) 
    print(result.stderr)

    if "Errors: none" not in output:
        return None

    f = open(dependency_file, "r")
    dependencies = f.read().splitlines()
    f.close()
    return dependencies

clean()

sources = get_sources()

marked_sources = set()
for s in sources:
    marked_sources.add(str(Path(s).resolve()))

while len(sources) > 0:
    source = sources.pop()
    dependencies = build_file(compile_command, source)

    if dependencies is None:
        print("Build error, quitting")
        exit()

    for file in dependencies:
        if not file.endswith(".h"):
            continue
            
        dep_source = get_header_source(file)

        if dep_source == None: # source file based on header file name not found
            continue

        dep_source = str(Path(dep_source).resolve()) # ensure capitalization is correct

        if dep_source in marked_sources: # dependency was already compiled
            continue

        marked_sources.add(dep_source)
        sources.append(dep_source)
        print(f"Adding dependency {dep_source}")

objects = get_object_files(marked_sources)

result = subprocess.run(" ".join(link_command + BLE_LIBS + objects), shell=True, text=True, capture_output=True)
print(result.stdout)
print(result.stderr)

subprocess.run(["hex2bin", f"{BIN_DIR}/{TARGET}.hex"])