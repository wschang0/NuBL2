tfm_split ..\..\..\..\..\..\trusted-firmware-m\build\unit_test\tfm_s.bin ..\..\..\..\..\..\trusted-firmware-m\build\unit_test\tfm_s.map
@REM tfm_split ..\..\..\..\..\..\trusted-firmware-m\build\secure_fw\tfm_s.bin ..\..\..\..\..\..\trusted-firmware-m\build\secure_fw\tfm_s.map
FwSign tfm_s_lr.bin+tfm_s_nsc.bin tfm_s_i.bin 0x7800 0x2c800
FwSign ..\..\..\..\..\..\trusted-firmware-m\build\app\tfm_ns.bin tfm_ns_i.bin 0x78C0
@REM FwSign ..\..\..\..\TF-M\tfm_ns\Nonsecure\KEIL\Objects\tfm_ns.bin tfm_ns_i.bin 0x78C0
