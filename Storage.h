#pragma once
#include "HL.Types.h"
#include "Model.h"


//-------------------------------------- EEPROM ------------------------------------//

#define EADDR_INFO     ((void *)0)  //���������EEPROM�����ַ
#define EADDR_CONFIG   ((void *)8)  //���õ�EEPROM�����ַ
#define EADDR_THROTTLE ((void *)32) //�������Ե�EEPROM�����ַ
#define EADDR_RUDDER   ((void *)40)
#define EADDR_PID      ((void *)64)

#define ESIZE_INFO     sizeof(Storage::StorageInfoStruct)
#define ESIZE_CONFIG   sizeof(Model::ConfigStruct)
#define ESIZE_THROTTLE sizeof(Model::PropertyStruct::ThrottleProperty)
#define ESIZE_RUDDER   sizeof(Model::PropertyStruct::RudderProperty)
#define ESIZE_PID_PART (sizeof(float) * 3) //PID�Ĺؼ�������С

#define EKEY (0b10101010)



/*
enum StoreFlag
{
	etStore_Config   = 0b0001,
	etStore_Throttle = 0b0010,
	etStore_Rudder   = 0b0100,
	etStore_PID      = 0b1000,
};
*/

/*
//���л���, ���ڽ���������Ϊ�ֽ����ݸ�ʽ
//Ҳ��߽����ݽ��Ϊ�����Ĺ���
class Serializer
{
	// *����ʲô
	// *���浽��
	// *��ô����
	static byte *GetData(Model::ConfigStruct &config)
	{

	}

	static byte *LoadData()
	{

	}


};
*/


class Storage
{
	static const u8 tb_src_offset[];
	static void *const tb_const_address[];
	static void *const tb_eep_address[];
	static const u8 tb_size[];

	struct StorageInfoStruct
	{
		u8 key;
		u8 cm_flags;
		u8 pid_flags;
	};

	static void StoreBytes(const void *from, void *to, u8 size);

	static void LoadBytes(void *to, const void *from, u8 size);

	public:
	static void UpdateFlags(u8 cm_flags, u8 pid_flags);

	static void ClearFlags(u8 cm_flags, u8 pid_flags);

	static bool DataExists(u8 &ptr_cm_flags, u8 &ptr_pid_flags);

	static void StoreParameter(Model &model, u8 id);

	static void LoadParameter(Model &model, u8 id);

	static void ResetParameter(Model &model, u8 id);


	static void StoreParameters(Model &model, u8 cm_flags);

	static u8 LoadParameters(Model &model, u8 cm_flags);

	static void ResetParameters(Model &model, u8 cm_flags);


	static void StorePID(PIDController pid[8], u8 index);

	static void LoadPID(PIDController pid[8], u8 index);

	static void ResetPID(PIDController pid[8], u8 index);


	static void StorePIDs(PIDController pid[8], u8 pid_flags);

	static u8 LoadPIDs(PIDController pid[8], u8 pid_flags);

	static void ResetPIDs(PIDController pid[8], u8 pid_flags);

	

};

