1��acc�ļ����ǵ����ٶȼƹ���������acc-stepcounter�Ǽ��ٶȼƴ��Ʋ�������������
2���������ʺ�MTKƽ̨android L��KK��
3�����Ʋ������ܣ���Ҫ��ƽ̨���ܡ�
	KK�汾��CUSTOM_KERNEL_STEP_COUNTER=yes	
	L�汾��CONFIG_CUSTOM_KERNEL_STEP_COUNTER=y
4���Ʋ���������Ҫ�򿪽ڵ�Ȩ�ޣ���init.mt6735.rc
	chmod 0660 /sys/class/misc/m_step_c_misc/step_cactive
	chmod 0660 /sys/class/misc/m_step_c_misc/step_cdelay
	chmod 0660 /sys/class/misc/m_step_c_misc/step_cbatch
	chmod 0660 /sys/class/misc/m_step_c_misc/step_cflush
	chown system system /sys/class/misc/m_step_c_misc/step_cactive
	chown system system /sys/class/misc/m_step_c_misc/step_cdelay
	chown system system /sys/class/misc/m_step_c_misc/step_cbatch
	chown system system /sys/class/misc/m_step_c_misc/step_cflush