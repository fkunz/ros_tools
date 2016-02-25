//=================================================================================================
// Copyright (c) 2015, Florian Kunz, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <fstream>
#include <iomanip>
#include <vector>

#include <ros/ros.h>

class BaseLogger
{
public:
    BaseLogger();
    ~BaseLogger();

    void init(std::vector<std::string> labels);

    template<class T>
    void log(std::pair<std::string, T> log_data);
    template<class T>
    void log(std::vector<T> log_vector);
    template<class T>
    void writeToFile(std::pair<std::string, T> log_data);
    template<class T>
    void writeToFile(std::vector<T> log_vector);

protected:
    template<class T>
    std::string convertToString(const T log_data);
    unsigned int getLableIndex(const std::string label);
    void writeLineToFile(const std::string log_line);
    void writeTempToFile();
    template<class T>
    void writeVectorToFile(const std::vector<T> log_vector);

private:
    std::string log_file_name_;

    std::vector<std::string> labels_;
    std::ofstream logfile_;
    unsigned int number_of_labels_;
    std::vector<std::string> temp_log_;
};

BaseLogger::BaseLogger()
{
    ros::NodeHandle private_nh("~");

    private_nh.param("log_file_name", log_file_name_, std::string("csv.log"));

    logfile_.open(log_file_name_.c_str());
    number_of_labels_ = 0;
}

BaseLogger::~BaseLogger()
{
    logfile_.close();
}

void BaseLogger::init(std::vector<std::string> labels)
{
    if (number_of_labels_ > 0)
    {
        // TODO(fkunz): handle double call to init.
        number_of_labels_ = 0;
    }
    labels_ = labels;

    while (number_of_labels_ < labels.size())
    {
        logfile_ << labels[number_of_labels_] << ", ";
        ++number_of_labels_;
    }

    logfile_ << std::fixed << std::setw( 24 ) << std::setprecision( 8 ) << std::setfill( ' ' );
    logfile_ << "\n";
}

template<class T>
void BaseLogger::log(std::pair<std::string, T> log_data)
{
    unsigned int label_index = getLableIndex(log_data.first);
    while (temp_log_.size() < label_index)
        temp_log_.push_back(std::string());
    if (temp_log_.size() == label_index)
        temp_log_.push_back(convertToString(log_data.second));
    else
        temp_log_[label_index] = log_data.second;
}

template<class T>
void BaseLogger::log(std::vector<T> log_vector)
{
    for (unsigned int i = 0; i < log_vector.size(); ++i)
    {
        temp_log_[i] = convertToString(log_vector[i]);
    }
}

template<class T>
void BaseLogger::writeToFile(std::pair<std::string, T> log_data)
{
    log(log_data);
    writeTempToFile();
}

template<class T>
void BaseLogger::writeToFile(std::vector<T> log_vector)
{

}

template<class T>
std::string BaseLogger::convertToString(const T log_data)
{
    std::stringstream string_stream;
    string_stream << log_data;
    return string_stream.str();
}

unsigned int BaseLogger::getLableIndex(const std::string label)
{
    unsigned int index = 0;
    while (labels_[index] != label && index < number_of_labels_)
        ++index;

    if (index == number_of_labels_)
        // TODO(fkunz): handle missing label.

    return index;
}

void BaseLogger::writeLineToFile(const std::string log_line)
{
    logfile_ << log_line << "\n";
}

void BaseLogger::writeTempToFile()
{
    writeVectorToFile(temp_log_);
}

template<class T>
void BaseLogger::writeVectorToFile(const std::vector<T> log_vector)
{
    std::stringstream log_line;
    for (unsigned int i = 0; i < log_vector.size(); ++i)
        log_line << log_vector[i] << ", ";

    writeLineToFile(log_line.str());
}
