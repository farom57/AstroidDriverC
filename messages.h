#include <cstdint>

// Message from telescope to the driver
class Status_message{
    public:
        static const int SIZE = 56;

        int32_t ms_count;
        int32_t step_ha;
        int32_t step_de;
        float ustep_ha;
        float ustep_de;
        float move_speed_ha;
        float move_speed_de;
        float power_ha;
        float power_de;
        uint16_t power_aux_1;
        uint16_t power_aux_2;
        uint16_t power_aux_3;
        // byte 42 not used
        int32_t step_focus;
        float ustep_focus;
        float move_speed_focus;
        uint8_t checksum;

        Status_message();
        bool set_buf(uint8_t const *buf);
        static bool verify(uint8_t const *buf);

        double getDE();
        double getHA();

};

// Message from the driver to the telescope
class Cmd_message{
    public:
        static const int SIZE = 32;

        float speed_ha;
        float speed_de;
        float power_ha;
        float power_de;
        uint16_t power_aux_1;
        uint16_t power_aux_2;
        uint16_t power_aux_3;
        float speed_focus;
        float power_focus;

        Cmd_message();
        void get_bytes(uint8_t *buf);

};
