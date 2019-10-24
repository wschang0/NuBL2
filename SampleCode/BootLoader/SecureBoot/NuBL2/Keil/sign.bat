tfm_split d:\MCU\TF-M\trusted-firmware-m\build\app\secure_fw\tfm_s.bin d:\MCU\TF-M\trusted-firmware-m\build\app\secure_fw\tfm_s.map
FwSign tfm_s_lr.bin+tfm_s_nsc.bin tfm_s_info.bin 0x7800
FwSign d:\MCU\TF-M\trusted-firmware-m\build\app\tfm_ns.bin tfm_ns_info.bin 0x78C0
@REM FwSign D:\MCU\TF-M\NuBL2\SampleCode\TF-M\tfm_ns\Nonsecure\KEIL\Objects\tfm_ns.bin tfm_ns_info.bin 0x78C0
