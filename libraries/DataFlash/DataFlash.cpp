#include <DataFlash.h>

// start functions pass straight through to backend:
void DataFlash_Class::WriteBlock(const void *pBuffer, uint16_t size) {
    backend->WriteBlock(pBuffer, size);
}
uint16_t DataFlash_Class::start_new_log() {
    return backend->start_new_log();
}

// change me to "DoTimeConsumingPreparations"?
void DataFlash_Class::EraseAll() {
    backend->EraseAll();
}
void DataFlash_Class::Prep_MinSpace() {
    backend->Prep_MinSpace();
}
// change me to "LoggingAvailable"?
bool DataFlash_Class::CardInserted(void) {
    return backend->CardInserted();
}
// remove me in favour of calling "DoTimeConsumingPreparations" all the time?
bool DataFlash_Class::NeedErase(void) {
    return backend->NeedErase();
}

uint16_t DataFlash_Class::find_last_log(void) {
    return backend->find_last_log();
}
void DataFlash_Class::get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) {
    backend->get_log_boundaries(log_num, start_page, end_page);
}
void DataFlash_Class::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) {
    backend->get_log_info(log_num, size, time_utc);
}
int16_t DataFlash_Class::get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) {
    return backend->get_log_data(log_num, page, offset, len, data);
}
uint16_t DataFlash_Class::get_num_logs(void) {
    return backend->get_num_logs();
}
void DataFlash_Class::Log_Fill_Format(const struct LogStructure *s, struct log_Format &pkt) {
    backend->Log_Fill_Format(s, pkt);
}

#ifndef DATAFLASH_NO_CLI
void DataFlash_Class::LogReadProcess(uint16_t log_num,
                                     uint16_t start_page, uint16_t end_page,
                                     void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                                     AP_HAL::BetterStream *port) {
    backend->LogReadProcess(log_num, start_page, end_page, print_mode, port);
}
void DataFlash_Class::DumpPageInfo(AP_HAL::BetterStream *port) {
    backend->DumpPageInfo(port);
}
void DataFlash_Class::ShowDeviceInfo(AP_HAL::BetterStream *port) {
    backend->ShowDeviceInfo(port);
}
void DataFlash_Class::ListAvailableLogs(AP_HAL::BetterStream *port) {
    backend->ListAvailableLogs(port);
}
#endif // DATAFLASH_NO_CLI

bool DataFlash_Class::logging_started(void) {
    return backend->logging_started();
}

void DataFlash_Class::EnableWrites(bool enable) {
    backend->EnableWrites(enable);
}

// for DataFlash_MAVLink
void DataFlash_Class::remote_log_block_status_msg(mavlink_message_t* msg) {
    backend->remote_log_block_status_msg(msg);
}
// end for DataFlash_MAVLink

// end functions pass straight through to backend
