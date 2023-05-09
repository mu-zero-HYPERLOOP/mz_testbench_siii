/*
 * EstimatedStateRegistry.hpp
 *
 *  Created on: May 4, 2023
 *      Author: OfficeLaptop
 */

#pragma once

class EstimatedStateRegistry{
public:
	static EstimatedStateRegistry& getInstance() {
		static EstimatedStateRegistry instance;
		return instance;
	}

	[[nodiscard]] inline float getPosition(){
		return m_position;
	}

	void setPosition(float position){
		m_position = position;
	}

private:
	explicit EstimatedStateRegistry(){

	}
	EstimatedStateRegistry(EstimatedStateRegistry&) = delete;
	EstimatedStateRegistry(EstimatedStateRegistry&&) = delete;
	EstimatedStateRegistry& operator=(EstimatedStateRegistry&) = delete;
	EstimatedStateRegistry& operator=(EstimatedStateRegistry&&) = delete;

	float m_position;
};
