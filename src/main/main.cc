#include "esp_system.h"
#include "driver/spi_master.h"
#include "nvs_flash.h"

#include "esp_cxx/event_manager.h"
#include "esp_cxx/logging.h"
#include "esp_cxx/ota.h"
#include "esp_cxx/wifi.h"

static const char *kTag = "cmwmic";

static void dump_chip_info() {
  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  ESP_LOGI(kTag, "This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
	 chip_info.cores,
	 (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
	 (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

  ESP_LOGI(kTag, "silicon revision %d, ", chip_info.revision);

  ESP_LOGI(kTag, "%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
	 (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

  fflush(stdout);
}

/*
static void dump_ota_boot_info() {
  const esp_partition_t *configured = esp_ota_get_boot_partition();
  const esp_partition_t *running = esp_ota_get_running_partition();

  if (configured != running) {
    ESP_LOGW(kTag, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
             configured->address, running->address);
    ESP_LOGW(kTag, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
  }
  ESP_LOGI(kTag, "Running partition type %d subtype %d (offset 0x%08x)",
           running->type, running->subtype, running->address);
}
*/

void configure_pcm1863(spi_device_handle_t spi) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));       // Zero out the transaction
  t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
  t.length = 16;                     // Command is 16 bits
  t.tx_data[0] = (0x0B << 1) | 1;
  t.tx_data[1] = 0;
  ret=spi_device_polling_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            // Should have had no issues.
  ESP_LOGE(kTag, "rxlength %d, received: %0x %0x", t.rxlength, t.rx_data[0], t.rx_data[1]);
}

void InitializeAdc() {
  esp_err_t ret;
  spi_device_handle_t spi;
  spi_bus_config_t buscfg ={
    .mosi_io_num = 13,
    .miso_io_num = 12,
    .sclk_io_num = 14,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 0
  };

  spi_device_interface_config_t devcfg = {
    .mode=3,                              // SPI mode 3.. I think.
    .clock_speed_hz=8*1000*1000,           // Spec requires min 100ns period, which is 10mhz. 8mhz is good neough.
    .spics_io_num = 15,                   // CS pin
    .queue_size=7,                          //We want to be able to queue 7 transactions at a time
  };

  // Initialize the SPI bus
  ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
  ESP_ERROR_CHECK(ret);

  // Attach the ADC to the SPI bus
  ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);

  //Initialize the LCD
  configure_pcm1863(spi);
}

extern "C" void app_main(void) {
  using namespace esp_cxx;
  dump_chip_info();
//  dump_ota_boot_info();

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

//  RunOtaWatchdog();

  // Setup Wifi access.
  // TODO(awong): move all this into a wifi object.
  if (!Wifi::GetSsid() || !Wifi::GetPassword()) {
    Wifi::SetSsid(ENV_SSID);
    Wifi::SetPassword(ENV_PASSWORD);
  }

  Wifi wifi;
  if (!wifi.ConnectToAP()) {
    ESP_LOGE(kTag, "Failed to connect to AP OR create a Setup Network.");
  }

  InitializeAdc();

  // Create controller.
  QueueSetEventManager controller_event_manager(100);  // TODO(awong): Size this.

  // Start all event managers.
  controller_event_manager.Loop();
  ESP_LOGE(kTag, "This should never be reached!");
}
