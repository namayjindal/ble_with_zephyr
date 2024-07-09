#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <iomanip>

struct SensorData {
    double timestamp;
    int index;
    double accel_x, accel_y, accel_z;
    double gyro_x, gyro_y, gyro_z;
    double battery;
    // Add any other columns that might be in your CSV
};

std::vector<SensorData> readSensorData(const std::string& filename) {
    std::vector<SensorData> data;
    std::ifstream file(filename);
    std::string line, header;
    
    // Read and store the header
    std::getline(file, header);
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        SensorData point;
        
        std::getline(iss, token, ',');
        point.timestamp = std::stod(token);
        
        std::getline(iss, token, ',');
        point.index = std::stoi(token);
        
        std::getline(iss, token, ',');
        point.accel_x = std::stod(token);
        
        std::getline(iss, token, ',');
        point.accel_y = std::stod(token);
        
        std::getline(iss, token, ',');
        point.accel_z = std::stod(token);
        
        std::getline(iss, token, ',');
        point.gyro_x = std::stod(token);
        
        std::getline(iss, token, ',');
        point.gyro_y = std::stod(token);
        
        std::getline(iss, token, ',');
        point.gyro_z = std::stod(token);
        
        std::getline(iss, token, ',');
        point.battery = std::stod(token);
        
        // Add reading of any other columns that might be in your CSV
        
        data.push_back(point);
    }
    
    return data;
}

double calculateMagnitude(const SensorData& point) {
    return std::sqrt(point.accel_x * point.accel_x + point.accel_y * point.accel_y + point.accel_z * point.accel_z);
}

std::vector<int> detectPeaks(const std::vector<SensorData>& data, int windowSize, double threshold, int minDistance) {
    std::vector<int> peakIndices;
    std::vector<double> movingAverage(data.size());
    
    // Calculate moving average
    for (int i = 0; i < data.size(); ++i) {
        double sum = 0;
        int count = 0;
        for (int j = std::max(0, i - windowSize / 2); j < std::min(static_cast<int>(data.size()), i + windowSize / 2 + 1); ++j) {
            sum += calculateMagnitude(data[j]);
            count++;
        }
        movingAverage[i] = sum / count;
    }
    
    // Detect peaks
    for (int i = 1; i < data.size() - 1; ++i) {
        double current = calculateMagnitude(data[i]);
        if (current > movingAverage[i] + threshold &&
            current > calculateMagnitude(data[i-1]) &&
            current > calculateMagnitude(data[i+1])) {
            
            // Check if the new peak is far enough from the last detected peak
            if (peakIndices.empty() || i - peakIndices.back() >= minDistance) {
                peakIndices.push_back(i);
            }
        }
    }
    
    return peakIndices;
}

void saveSegmentToCSV(const std::vector<SensorData>& data, int start, int end, int segmentNumber, const std::string& header) {
    std::ostringstream filename;
    filename << "segment_" << segmentNumber << ".csv";
    std::ofstream outFile(filename.str(), std::ios::out | std::ios::trunc);
    
    // Write the original header
    outFile << header << "\r\n";
    
    // Start writing data immediately after the header, no empty row
    for (int i = start; i <= end; ++i) {
        outFile << std::fixed << std::setprecision(3)
                << data[i].timestamp << ","
                << data[i].index << ","
                << data[i].accel_x << ","
                << data[i].accel_y << ","
                << data[i].accel_z << ","
                << data[i].gyro_x << ","
                << data[i].gyro_y << ","
                << data[i].gyro_z << ","
                << data[i].battery << "\r\n";
    }
    
    outFile.close();
}


int main() {
    std::string filename = "sensor_data.csv";
    std::vector<SensorData> sensorData = readSensorData(filename);
    int windowSize = 20;  // Adjust this value based on your data
    double threshold = 0.75;  // Adjust this value based on your data
    int minDistance = 5;  // Minimum distance between peaks
    
    // Read the header from the original file
    std::ifstream originalFile(filename);
    std::string header;
    std::getline(originalFile, header);
    originalFile.close();
    
    std::vector<int> peakIndices = detectPeaks(sensorData, windowSize, threshold, minDistance);
    
    std::cout << "Peaks detected at the following indices:" << std::endl;
    for (const auto& index : peakIndices) {
        std::cout << index << " (Timestamp: " << sensorData[index].timestamp << ")" << std::endl;
    }
    
    // Extract segments and save to CSV files
    for (int i = 0; i < peakIndices.size() - 1; ++i) {
        int start = peakIndices[i];
        int end = peakIndices[i+1] - 1;
        saveSegmentToCSV(sensorData, start, end, i+1, header);
        std::cout << "Saved segment " << i+1 << " (from index " << start << " to " << end << ")" << std::endl;
    }
    
    return 0;
}
