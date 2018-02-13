class Network
{
public:
    static void init();

    static void sendPacket(ip_t ip, unsigned char *buf, size_t buf_len);
};
