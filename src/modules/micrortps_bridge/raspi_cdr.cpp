#include <iostream>
#include <fastcdr/Cdr.h>
#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>

class raspi
{
	public:
		raspi();
		void deserialize(char data_buffer[], size_t len);
		void serialize();
		float get_m_value();
	private:
		float m_value;

		char data_buffer[1000] = {};
		uint16_t length;
		char* ptr_data_buffer;
};

raspi::raspi()
{	
	length = 0;
	m_value = 0;
	ptr_data_buffer = data_buffer;
}

void raspi::deserialize(char data_buffer[], size_t len)
{
	eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
	eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
	cdr_des >> m_value;
}

void raspi::serialize()
{	
	len = sizeof(data_buffer)/sizeof(data_buffer[0]) ;
	eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
	eprosima::fastcdr::Cdr scdr(cdrbuffer);
	scdr << m_value;
	length = scdr.getSerializedDataLength();
}

float raspi::get_m_value()
{
	return m_value;
}

char* raspi::get_serialize_data()
{
	return ptr_data_buffer;
}

uint16_t raspi::get_length()
{
	return length;
}
extern "C"
{
	raspi* raspi_new(){return new raspi();}
	void raspi_deserialize(raspi* raspi_t,char data_buffer[],size_t len){raspi_t->deserialize(data_buffer,len);}
	void raspi_serialize(raspi* raspi_t){raspi_t->serialize();}

	float get_m_value(raspi* raspi_t){return raspi_t->get_m_value();}
	char* get_serialize_data(raspi* raspi_t){return raspi_t->get_serialize_data();}
	uint16_t get_length(raspi* raspi_t){return raspi_t->get_length();}
}