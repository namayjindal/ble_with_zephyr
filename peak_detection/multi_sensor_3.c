#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <numeric>

struct SensorData {
    double right_hand_timestamp;
    int right_hand_index;
    double right_hand_accel_x, right_hand_accel_y, right_hand_accel_z;
    double right_hand_gyro_x, right_hand_gyro_y, right_hand_gyro_z;
    double right_hand_battery;

    double left_hand_timestamp;
    int left_hand_index;
    double left_hand_accel_x, left_hand_accel_y, left_hand_accel_z;
    double left_hand_gyro_x, left_hand_gyro_y, left_hand_gyro_z;
    double left_hand_battery;
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
        point.right_hand_timestamp = std::stod(token);
        
        std::getline(iss, token, ',');
        point.right_hand_index = std::stoi(token);
        
        std::getline(iss, token, ',');
        point.right_hand_accel_x = std::stod(token);
        
        std::getline(iss, token, ',');
        point.right_hand_accel_y = std::stod(token);
        
        std::getline(iss, token, ',');
        point.right_hand_accel_z = std::stod(token);
        
        std::getline(iss, token, ',');
        point.right_hand_gyro_x = std::stod(token);
        
        std::getline(iss, token, ',');
        point.right_hand_gyro_y = std::stod(token);
        
        std::getline(iss, token, ',');
        point.right_hand_gyro_z = std::stod(token);
        
        std::getline(iss, token, ',');
        point.right_hand_battery = std::stod(token);
        
        std::getline(iss, token, ',');
        point.left_hand_timestamp = std::stod(token);
        
        std::getline(iss, token, ',');
        point.left_hand_index = std::stoi(token);
        
        std::getline(iss, token, ',');
        point.left_hand_accel_x = std::stod(token);
        
        std::getline(iss, token, ',');
        point.left_hand_accel_y = std::stod(token);
        
        std::getline(iss, token, ',');
        point.left_hand_accel_z = std::stod(token);
        
        std::getline(iss, token, ',');
        point.left_hand_gyro_x = std::stod(token);
        
        std::getline(iss, token, ',');
        point.left_hand_gyro_y = std::stod(token);
        
        std::getline(iss, token, ',');
        point.left_hand_gyro_z = std::stod(token);
        
        std::getline(iss, token, ',');
        point.left_hand_battery = std::stod(token);
        
        data.push_back(point);
    }
    
    return data;
}

double calculateMagnitude(double accel_x, double accel_y, double accel_z) {
    return std::sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
}

std::vector<double> movingAverage(const std::vector<double>& signal, int windowSize) {
    std::vector<double> result(signal.size());
    for (int i = 0; i < signal.size(); ++i) {
        int start = std::max(0, i - windowSize / 2);
        int end = std::min(static_cast<int>(signal.size()), i + windowSize / 2 + 1);
        result[i] = std::accumulate(signal.begin() + start, signal.begin() + end, 0.0) / (end - start);
    }
    return result;
}

std::vector<int> detectPeaks(const std::vector<SensorData>& data, int windowSize, double sensitivityFactor, int minDistance) {
    std::vector<int> peakIndices;
    std::vector<double> magnitudes(data.size());
    
    // Calculate accelerometer magnitudes
    for (int i = 0; i < data.size(); ++i) {
        magnitudes[i] = calculateMagnitude(data[i].right_hand_accel_x, data[i].right_hand_accel_y, data[i].right_hand_accel_z);
    }
    
    // Calculate moving average
    std::vector<double> movingAvg = movingAverage(magnitudes, windowSize);
    
    // Calculate standard deviation of magnitudes
    double mean = std::accumulate(magnitudes.begin(), magnitudes.end(), 0.0) / magnitudes.size();
    double sqSum = std::inner_product(magnitudes.begin(), magnitudes.end(), magnitudes.begin(), 0.0);
    double stdDev = std::sqrt(sqSum / magnitudes.size() - mean * mean);
    
    // Detect peaks using dynamic threshold based on moving average and standard deviation
    double threshold = sensitivityFactor * stdDev;
    for (int i = 1; i < data.size() - 1; ++i) {
        double current = magnitudes[i];
        if (current > movingAvg[i] + threshold &&
            current > magnitudes[i-1] &&
            current > magnitudes[i+1]) {
            
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
    filename << "actual_test/10/segment_" << segmentNumber << ".csv";
    std::ofstream outFile(filename.str(), std::ios::out | std::ios::trunc);
    
    // Write the original header
    outFile << header << "\n";
    
    // Start writing data immediately after the header, no empty row
    for (int i = start; i <= end; ++i) {
        outFile << std::fixed << std::setprecision(3)
                << data[i].right_hand_timestamp << ","
                << data[i].right_hand_index << ","
                << data[i].right_hand_accel_x << ","
                << data[i].right_hand_accel_y << ","
                << data[i].right_hand_accel_z << ","
                << data[i].right_hand_gyro_x << ","
                << data[i].right_hand_gyro_y << ","
                << data[i].right_hand_gyro_z << ","
                << data[i].right_hand_battery << ","
                << data[i].left_hand_timestamp << ","
                << data[i].left_hand_index << ","
                << data[i].left_hand_accel_x << ","
                << data[i].left_hand_accel_y << ","
                << data[i].left_hand_accel_z << ","
                << data[i].left_hand_gyro_x << ","
                << data[i].left_hand_gyro_y << ","
                << data[i].left_hand_gyro_z << ","
                << data[i].left_hand_battery << "\r\n";
    }
    
    outFile.close();
}

int main() {
    std::string filename = "up_down_10.csv";
    std::vector<SensorData> sensorData = readSensorData(filename);
    int windowSize = 20;  // Adjust this value based on your data
    double sensitivityFactor = 0.85;  // Adjust this value to change sensitivity (lower = more sensitive)
    int minDistance = 10;  // Minimum distance between peaks
    
    // Read the header from the original file
    std::ifstream originalFile(filename);
    std::string header;
    std::getline(originalFile, header);
    originalFile.close();
    
    std::vector<int> peakIndices = detectPeaks(sensorData, windowSize, sensitivityFactor, minDistance);
    
    std::cout << "Peaks detected: " << peakIndices.size() << std::endl;
    std::cout << "Peaks detected at the following indices:" << std::endl;
    for (const auto& index : peakIndices) {
        std::cout << index << " (Right hand timestamp: " << sensorData[index].right_hand_timestamp 
                  << ", Left hand timestamp: " << sensorData[index].left_hand_timestamp << ")" << std::endl;
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
