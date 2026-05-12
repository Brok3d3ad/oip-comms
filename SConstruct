#!/usr/bin/env python
import os
import sys

from methods import print_error


libname = "OIP-COMMS"
projectdir = "demo"

localEnv = Environment(tools=["default"], PLATFORM="")

customs = ["custom.py"]
customs = [os.path.abspath(path) for path in customs]

opts = Variables(customs, ARGUMENTS)
opts.Update(localEnv)

Help(opts.GenerateHelpText(localEnv))

env = localEnv.Clone()

submodule_initialized = False
dir_name = 'godot-cpp'
if os.path.isdir(dir_name):
    if os.listdir(dir_name):
        submodule_initialized = True

if not submodule_initialized:
    print_error("""godot-cpp is not available within this folder, as Git submodules haven't been initialized.
Run the following command to download godot-cpp:

    git submodule update --init --recursive""")
    sys.exit(1)

env = SConscript("godot-cpp/SConstruct", {"env": env, "customs": customs})

env.Append(CPPPATH=["src/", "ads/AdsLib/"])
env.Append(LIBPATH=["lib/"])
env.Append(LIBS=["plctag", "open62541"])
env.Append(CPPDEFINES=[("CONFIG_DEFAULT_LOGLEVEL", "1")])

# Local TC3 4026 connections require the TwinCAT variant; TC3 4026 enforces
# Secure ADS by default and silently drops requests from the standalone
# library (manifests as 0x745 sync timeout). Standalone variant works fine
# for remote TwinCAT systems. Auto-detect by probing for TcAdsDef.h; without
# TwinCAT installed the build still succeeds via standalone.
ads_sources_dir = "ads/AdsLib/standalone"
beckhoff_ads_root = "C:/Program Files (x86)/Beckhoff/TwinCAT/AdsApi/TcAdsDll"
have_twincat_sdk = (
    env["platform"] == "windows"
    and os.path.isfile(beckhoff_ads_root + "/Include/TcAdsDef.h")
)

if env["platform"] == "windows":
    env.Append(CXXFLAGS=["/MT", "/EHsc"])
    env.Append(LIBS=["ws2_32"])
    if have_twincat_sdk:
        print("ADS variant: TwinCAT (TcAdsDll detected at " + beckhoff_ads_root + ")")
        env.Append(CPPDEFINES=["USE_TWINCAT_ROUTER"])
        env.Append(CPPPATH=[beckhoff_ads_root + "/Include"])
        env.Append(LIBPATH=[beckhoff_ads_root + "/Lib/x64"])
        env.Append(LIBS=["TcAdsDll", "delayimp", "Advapi32"])
        # Delay-load so preload_tc_ads_dll() can resolve TcAdsDll from its
        # install path before any import is touched — works around stale PATH
        # in host processes started before TwinCAT was installed.
        env.Append(LINKFLAGS=["/DELAYLOAD:TcAdsDll.dll"])
        ads_sources_dir = "ads/AdsLib/TwinCAT"
    else:
        print("ADS variant: standalone (TcAdsDll not found at " + beckhoff_ads_root + ")")
else:
    env.Append(LINKFLAGS=["-static"])
sources = (
    Glob("src/*.cpp")
    + Glob("ads/AdsLib/*.cpp")
    + Glob("ads/AdsLib/bhf/*.cpp")
    + Glob(ads_sources_dir + "/*.cpp")
)

if env["target"] in ["editor", "template_debug"]:
    try:
        doc_data = env.GodotCPPDocData("src/gen/doc_data.gen.cpp", source=Glob("doc_classes/*.xml"))
        sources.append(doc_data)
    except AttributeError:
        print("Not including class reference as we're targeting a pre-4.3 baseline.")

file = "{}{}{}".format(libname, env["suffix"], env["SHLIBSUFFIX"])
filepath = ""

if env["platform"] == "macos" or env["platform"] == "ios":
    filepath = "{}.framework/".format(env["platform"])
    file = "{}{}".format(libname, env["suffix"])

libraryfile = "bin/{}/{}{}".format(env["platform"], filepath, file)
library = env.SharedLibrary(
    libraryfile,
    source=sources,
)

copy = env.InstallAs("{}/bin/{}/{}lib{}".format(projectdir, env["platform"], filepath, file), library)

default_args = [library, copy]
Default(*default_args)
