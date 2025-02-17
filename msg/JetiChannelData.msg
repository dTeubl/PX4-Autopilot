uint64 timestamp       # time since system start (microseconds)

uint8 head1            # first header byte
uint8 head2            # second header byte

uint8 len              # length of package
uint8 packet_id        # packet id
uint8 data_type          # data identifier
uint8 sub_len          # length of data block

uint16 crc16           # CRC16 CCITT

# TOPICS jeti_channel_data
