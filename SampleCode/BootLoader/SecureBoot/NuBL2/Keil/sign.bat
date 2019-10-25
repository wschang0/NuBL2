tfm_split ..\..\..\..\..\..\trusted-firmware-m\build\app\secure_fw\tfm_s.bin ..\..\..\..\..\..\trusted-firmware-m\build\app\secure_fw\tfm_s.map
FwSign tfm_s_lr.bin+tfm_s_nsc.bin tfm_s_info.bin 0x7800
FwSign ..\..\..\..\..\..\trusted-firmware-m\build\app\tfm_ns.bin tfm_ns_info.bin 0x78C0
@REM FwSign ..\..\..\..\TF-M\tfm_ns\Nonsecure\KEIL\Objects\tfm_ns.bin tfm_ns_info.bin 0x78C0
