CREATE TABLE `esp32_table_test` (
    `id` varchar(255) NOT NULL,
	sens_id int(10)
    `vibration` float(10,2) NOT NULL,
    `temperature` float(10,2) NOT NULL,
    `charge` float(10,2) NOT NULL,
    date date,
    PRIMARY KEY (`id`)
)
