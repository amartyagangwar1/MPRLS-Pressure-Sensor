#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

static const struct i2c_dt_spec mprls = I2C_DT_SPEC_GET(DT_NODELABEL(mprls));

int main(void){
    if (!device_is_ready(mprls.bus)) {
        printk("I2C bus %s is not ready!\n",mprls.bus->name);
        return;
    }

    printk("I2C ready, addr=0x%x\n", mprls.addr);

    while (1) {
        /*tell sensor to measure*/
        uint8_t config[3] = {0xAA, 0x00, 0x00};
        int ret = i2c_write_dt(&mprls, config, sizeof(config));
        if(ret < 0){
            printk("Failed to write to I2C device address %x.\n", mprls.addr);
        } 

        /*wait for measurement to be ready*/
        k_msleep(5);
        
        /*read measurement*/
        uint8_t data[4];    //status byte, pressure[23:16], pressure[15:8], pressure[7:0]
        ret = i2c_read_dt(&mprls, data, sizeof(data));
        if(ret != 0){
            printk("Failed to read from I2C device address %x. \n", mprls.addr);
        } 

        /*process and print measurement*/
        uint32_t raw = 0;
        float pressure_psi = 0.0f; 


        if(!(data[0] & 0b00100000)){
            raw = (data[1] << 16) + (data[2] << 8) + data[3];
            pressure_psi = ((float)raw - 1677722.0f) * 25.0f / (15099494.0f - 1677722.0f);
        }

        int whole = (int)pressure_psi;
        int frac  = (int)((pressure_psi - whole) * 100.0f);

        printk("%d.%02d\n", whole,frac);

        /*wait so the rate is correct (10 Hz)*/
        k_msleep(95);
    }
}
