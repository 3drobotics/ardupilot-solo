/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */

#ifndef DataFlash_MAVLink_h
#define DataFlash_MAVLink_h

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <systemlib/perf_counter.h>
#else
#define perf_begin(x)
#define perf_end(x)
#define perf_count(x)
#endif

#define NUM_BUFFER_BLOCKS 80
#define BUFFER_BLOCK_SIZE 200


class DataFlash_MAVLink : public DataFlash_Class
{
public:
    // constructor
    DataFlash_MAVLink(mavlink_channel_t chan);

    // initialisation
    void Init(const struct LogStructure *structure, uint8_t num_types);


    /* Write a block of data at current offset */
    void WriteBlock(const void *pBuffer, uint16_t size);

    // initialisation
    bool CardInserted(void) { return true; }

    // erase handling
    bool NeedErase(void){ return false; }
    void EraseAll(){}

    // high level interface
    uint16_t find_last_log(void) { return 0; }
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) {}
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) {}
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) { return 0; }
    uint16_t get_num_logs(void){return 0;}
    
    void LogReadProcess(uint16_t log_num,
                        uint16_t start_page, uint16_t end_page, 
                        void (*printMode)(AP_HAL::BetterStream *port, uint8_t mode),
                        AP_HAL::BetterStream *port) {}
    void DumpPageInfo(AP_HAL::BetterStream *port) {}
    void ShowDeviceInfo(AP_HAL::BetterStream *port) {}
    void ListAvailableLogs(AP_HAL::BetterStream *port) {}
    void send_log_block(uint32_t block_address);
    void handle_ack(uint32_t block_num);
    void handle_retry(uint32_t block_num);
    void set_channel(mavlink_channel_t chan);
private:

    mavlink_channel_t _chan;
    bool _initialised;
    bool _only_critical_blocks;

    uint16_t _total_blocks;
    const uint16_t _block_max_size;
    uint32_t _latest_block_num;
    uint8_t _cur_block_address;
    uint16_t _latest_block_len;
    uint32_t _last_response_time;

    // write buffer
    uint8_t _buf[NUM_BUFFER_BLOCKS][BUFFER_BLOCK_SIZE];
    uint32_t _block_num[NUM_BUFFER_BLOCKS];
    bool _is_critical_block[NUM_BUFFER_BLOCKS];
    struct {
        // socket to telem2 on aircraft
        bool connected;
        uint8_t system_id;
        uint8_t component_id;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink;

    uint16_t start_new_log(void) { return 0; }
    void ReadBlock(void *pkt, uint16_t size) {}
    int8_t next_block_address();
    bool _buffer_empty();
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // performance counters
    perf_counter_t  _perf_write;
    perf_counter_t  _perf_errors;
    perf_counter_t  _perf_overruns;
#endif
};


#endif // DataFlash_MAVLink_h

