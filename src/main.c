#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

static uint32_t ble_pressure = 0;

static ssize_t read_pressure(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset,&ble_pressure, sizeof(ble_pressure));
}

BT_GATT_SERVICE_DEFINE(pressure_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0x180F)), BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(0x2A19),
                       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, read_pressure, NULL, NULL), BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

static const struct i2c_dt_spec mprls = I2C_DT_SPEC_GET(DT_NODELABEL(mprls));

int main(void){

    //BLE
    printk("BOOT\n");
    int err = bt_enable(NULL);
    if (err) {
        printk("BLE init failed (%d)\n", err);
        return 0;
    }

    struct bt_data ad[] = {BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR), BT_DATA(BT_DATA_NAME_COMPLETE,
                           CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),};

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed (%d)\n", err);
        return 0;
    }
    printk("Advertising started\n");


    if (!device_is_ready(mprls.bus)) {
        printk("I2C bus %s is not ready!\n",mprls.bus->name);
        return;
    }

    //I2C & Adv
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

        /*
        int whole = (int)pressure_psi;
        int frac  = (int)((pressure_psi - whole) * 100.0f);

        printk("%d.%02d\n", whole,frac); 
        */

        ble_pressure = (uint32_t)(pressure_psi * 1000.0f); // mPSI
        bt_gatt_notify(NULL, &pressure_svc.attrs[1],&ble_pressure, sizeof(ble_pressure));

        /*wait so the rate is correct (10 Hz)*/
        k_msleep(95);
    }
}
