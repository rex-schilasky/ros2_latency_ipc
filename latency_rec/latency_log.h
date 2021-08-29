#pragma once

#include <string>
#include <vector>

void evaluate(std::vector<long long>& lat_arr_, size_t rec_size_, size_t warmups_, std::string& log_file_);
void log2file(std::vector<long long>& lat_arr_, size_t rec_size_, std::string& log_file_);
