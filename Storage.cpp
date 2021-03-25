#include "Storage.h"
//#include "Model.h"
//#include "EEPROM.h"
#include "avr/eeprom.h"
#include "avr/pgmspace.h"

//--------- const default parameters ----------//

// 无人机配置参数
PROGMEM const Model::ConfigStruct g_config = { 16, true, true };

PROGMEM const Model::LogStruct g_log = { 0b0000, 0b000, false };

//13B
//无人机初始属性 (可以设为static的量)
PROGMEM const Model::PropertyStruct g_property =
{
	{ 1000, 1000 }, 
	{ { 1500, 1500, 1500, 1500 }, 16 } 
};

//96B
PROGMEM const float g_pids[8][3] =
{
	{ 0, 0, 0 }, { 0, 0, 0 },
	{ 0.2f, 0, 0 }, { 1.0f, 0, 0 },  //rx.ki=> 1-4 //rx.pd=> 0-1
	{ 0.2f, 0, 0 }, { 1.0f, 0, 0 },
	{ 0.2f, 0, 0 }, { 2.0f, 1.0f, 0 },
};


//---------------- storage table --------------//

const u8 Storage::tb_src_offset[] =
{
	(u8) & ((Model *)0)->config,
	(u8) & ((Model *)0)->property.throttle,
	(u8) & ((Model *)0)->property.rudder,
	//log
};

void *const Storage::tb_const_address[] =
{
	(void *)&g_config,
	(void *)&g_property.throttle,
	(void *)&g_property.rudder,
	//(void *)&g_log,
};

void *const Storage::tb_eep_address[] = { EADDR_CONFIG, EADDR_THROTTLE, EADDR_RUDDER };

const u8 Storage::tb_size[] = { ESIZE_CONFIG, ESIZE_THROTTLE, ESIZE_RUDDER };




void Storage::StoreBytes(const void *from, void *to, u8 size)
{
	eeprom_busy_wait();
	eeprom_update_block(from, to, size);
	//return true;
}

void Storage::LoadBytes(void *to, const void *from, u8 size)
{
	eeprom_busy_wait();
	eeprom_read_block(to, from, size);
	//return true;
}

void Storage::UpdateFlags(u8 cm_flags, u8 pid_flags)
{
	if(cm_flags == 0 && pid_flags == 0) return;

	StorageInfoStruct info;
	LoadBytes(&info, EADDR_INFO, ESIZE_INFO);

	if(info.key != EKEY) info.key = EKEY;
	info.cm_flags |= cm_flags;
	info.pid_flags |= pid_flags;

	StoreBytes(&info, EADDR_INFO, ESIZE_INFO);
}

void Storage::ClearFlags(u8 cm_flags, u8 pid_flags)
{
	StorageInfoStruct info;
	LoadBytes(&info, EADDR_INFO, ESIZE_INFO);

	if(info.key != EKEY) info.key = EKEY;
	info.cm_flags &= ~cm_flags;
	info.pid_flags &= ~pid_flags;

	StoreBytes(&info, EADDR_INFO, ESIZE_INFO);

	LOGF("Delete Stored: ");
	LOG(cm_flags, BIN);
	LOGF("  PID: ");
	LOGLN(pid_flags, BIN);
	Beeper::Beep(0b11110101);
}


bool Storage::DataExists(u8 &ptr_cm_flags, u8 &ptr_pid_flags)
{
	StorageInfoStruct info;
	LoadBytes(&info, EADDR_INFO, ESIZE_INFO);

	if(info.key != EKEY)
	{
		ptr_cm_flags = 0;
		ptr_pid_flags = 0;
		return false;
	}
	else
	{
		ptr_cm_flags = info.cm_flags;
		ptr_pid_flags = info.pid_flags;
		return true;
	}
}


void Storage::StoreParameter(Model &model, u8 id)
{
	const void *source = (byte *)&model + tb_src_offset[id];
	void *destination = tb_eep_address[id];
	u8 size = tb_size[id];
	StoreBytes(source, destination, size);

	LOGF("Store parameters[");
	LOG(id);
	LOGF("]\n");
	Beeper::Beep(0b1101);
}

void Storage::LoadParameter(Model &model, u8 id)
{
	const void *source = tb_eep_address[id];
	void *destination = (byte *)&model + tb_src_offset[id];
	u8 size = tb_size[id];
	LoadBytes(destination, source, size);

	LOGF("Load parameters[");
	LOG(id);
	LOGF("]\n");
	Beeper::Beep(0b1001);
}

void Storage::ResetParameter(Model &model, u8 id)
{
	const byte *source = (byte *)tb_const_address[id];
	byte *destination = (byte *)&model + tb_src_offset[id];
	u8 size = tb_size[id];

	LOGF("Reset parameters[");
	LOG(id);
	LOGF("]: ");

	while(size != 0)
	{
		*destination = pgm_read_byte(source);
		LOG(*destination, BIN);
		LOG(" ");
		destination++;
		source++;
		size--;
	}
	LOG("\n");

	Beeper::Beep(0b101001);
}


void Storage::StoreParameters(Model &model, u8 cm_flags)
{
	for(u8 i = 0; i < 3; i++)
	{
		if(cm_flags & (1 << i))
		{
			StoreParameter(model, i);
		}
	}

	UpdateFlags(cm_flags, 0);
}

u8 Storage::LoadParameters(Model &model, u8 cm_flags)
{
	u8 cm_es, pid_es;
	DataExists(cm_es, pid_es);
	cm_flags &= cm_es;
	if(cm_flags == 0) return 0x00;

	for(u8 i = 0; i < 3; i++)
	{
		if(cm_flags & (1 << i))
		{
			LoadParameter(model, i);
		}
	}
	return cm_flags;
}

void Storage::ResetParameters(Model &model, u8 cm_flags)
{
	for(u8 i = 0; i < 3; i++)
	{
		if(cm_flags & (1 << i))
		{
			ResetParameter(model, i);
		}
	}
}



void Storage::StorePID(PIDController pid[8], u8 index)
{
	float values[3];
	values[0] = pid[index].kp;
	values[1] = pid[index].ki;
	values[2] = pid[index].kd;

	void *address = (void *)((u16)EADDR_PID + index * ESIZE_PID_PART);
	StoreBytes(values, address, ESIZE_PID_PART);

	LOGF("Store PID[");
	LOG(index);
	LOGF("]\n");
	Beeper::Beep(0b100101, 6);
}

void Storage::LoadPID(PIDController pid[8], u8 index)
{
	float values[3];
	void *address = (void *)((u16)EADDR_PID + index * ESIZE_PID_PART);
	LoadBytes(values , address, ESIZE_PID_PART);
	pid[index].kp = values[0];
	pid[index].ki = values[1];
	pid[index].kd = values[2];

	LOGF("Load PID[");
	LOG(index);
	LOGF("]\n");
	Beeper::Beep(0b110101, 6);
}

void Storage::ResetPID(PIDController pid[8], u8 index)
{
	pid[index].kp = pgm_read_float(&g_pids[index][0]);
	pid[index].ki = pgm_read_float(&g_pids[index][1]);
	pid[index].kd = pgm_read_float(&g_pids[index][2]);

	LOGF("Reset PID[");
	LOG(index);
	LOGF("]\n");
	Beeper::Beep(0b10110101);
}


void Storage::StorePIDs(PIDController pid[8], u8 pid_flags)
{
	for(u8 i = 0; i < 8; i++)
	{
		if(pid_flags & (1 << i))
		{
			StorePID(pid, i);
		}
	}

	UpdateFlags(0, pid_flags);

	//Beeper::Beep(0b11000101);
}

u8 Storage::LoadPIDs(PIDController pid[8], u8 pid_flags)
{
	u8 cm_es, pid_es;
	DataExists(cm_es, pid_es);
	pid_flags &= pid_es;
	if(pid_flags == 0) return 0x00;

	for(u8 i = 0; i < 8; i++)
	{
		if(pid_flags & (1 << i))
		{
			LoadPID(pid, i);
		}
	}

	//Beeper::Beep(0b01000101);
	return pid_flags;
}

void Storage::ResetPIDs(PIDController pid[8], u8 pid_flags)
{
	for(u8 i = 0; i < 8; i++)
	{
		if(pid_flags & (1 << i))
		{
			ResetPID(pid, i);
		}
	}
}


