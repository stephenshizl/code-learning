1����qma6981.c��qma6981.h����Ŀ¼�С�
2����kernel/arch/arm64/configs/xxx_defconfig�����CONFIG_SENSORS_QMA6981=y��
ͬʱ��kernel\drivers\input\misc�е�Kconfig��Makefile�����Ӧ��䡣
3��kernel/arch/arm64/boot/dts/qcom�е��豸���ļ�dtsi�����qma6981������ݣ��磺
qma6981@12 {
		
compatible = "qst,qma6981";

reg = <0x12>;

vdd-supply = <&pm8916_l17>;

vddio-supply = <&pm8916_l6>;

qst,layout = <5>;

qst,poll_report = <1>;

};
