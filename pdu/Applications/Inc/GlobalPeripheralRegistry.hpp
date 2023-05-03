/*
 * GlobalPeripheralRegistry.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once

class GlobalPeripheralRegistry {
private:
	GlobalPeripheralRegistry() {

	}
	GlobalPeripheralRegistry(GlobalPeripheralRegistry&) = delete;
	GlobalPeripheralRegistry(GlobalPeripheralRegistry&&) = delete;
	GlobalPeripheralRegistry& operator=(GlobalPeripheralRegistry&) = delete;
	GlobalPeripheralRegistry& operator=(GlobalPeripheralRegistry&&) = delete;
public:
	[[nodiscard]] static inline GlobalPeripheralRegistry& getInstance() {
		static GlobalPeripheralRegistry instance;
		return instance;
	}

private:
};
