#pragma once

#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <chrono>
#include <memory>
#include <iostream>

namespace utils {

    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

    class Timing {
        std::chrono::steady_clock::time_point begin, end;
        double mean_time_temp = 0.0;
        double max_time = 1e-12;

    public:
        double elapsed_time = 0.0;
        void tic() { begin = std::chrono::steady_clock::now();}
        void toc() { end = std::chrono::steady_clock::now(); }
        void update() {
          elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()
                         / 1000.0; // divided to get milliseconds with microsecond accuracy
          std::cout << "iteration time duration = " << elapsed_time << " [ms]" << std::endl;
          mean_time_temp += elapsed_time;
          max_time = std::max(elapsed_time, max_time);
        }
        void finish(const int Nsim) const {
          double mean_time = mean_time_temp / Nsim;
          std::cout << "mean time for one iteration =" << mean_time << " [ms]" << std::endl;
          std::cout << "maximum time for one iteration =" << max_time << " [ms]" << std::endl;
        }
    };


    class ContainerBase{
    public:
        virtual void log() = 0;
        virtual ~ContainerBase() = default;
    protected:
        std::ofstream file;
        void createFile(const std::string & dir, const std::string & name){
          std::cout << "created LOG in " << dir << "/" << name << std::endl;
          file = std::ofstream(dir + "/" +  name, std::ofstream::out);
        }
    };

    template <typename Derived>
    class EigenContainer : public ContainerBase {
    public:
        EigenContainer(const Eigen::DenseBase<Derived> & var_to_be_logged, const std::string & dir, const std::string & name, const bool transpose):
            var_ptr(var_to_be_logged), transpose(transpose) {
          createFile(dir, name);
        }
        ~EigenContainer() { file.close();}

        void log() {
          if(transpose) file << var_ptr.transpose().format(CSVFormat) << std::endl;
          else file << var_ptr.format(CSVFormat) << std::endl;
        }

    private:
        const Eigen::DenseBase<Derived> & var_ptr;
        bool transpose = false;
    };

    template <typename Derived>
    class ScalarContainer : public ContainerBase {
    public:
        ScalarContainer(const Derived & var_to_be_logged, const std::string & dir, const std::string & name):
            var_ptr(var_to_be_logged) {
          createFile(dir, name);
        }
        ~ScalarContainer() {file.close();}

        void log() {file << var_ptr << std::endl;}

    private:
        const Derived & var_ptr;

    };


    class Logger {
    public:

        Logger() : loggingDirectory(realpath("./", nullptr)) {}
        Logger(const std::string_view & loggingDir) : loggingDirectory(loggingDir) {}
        
        /// Initializer for Eigen types containers with no transpose() applied
        template <typename Derived>
        void add(const Eigen::DenseBase<Derived> & var_to_be_logged, const std::string & name) {
          auto container_ptr = std::make_shared<EigenContainer<Derived>>(var_to_be_logged, loggingDirectory, name, false);
          list.push_back(container_ptr);
        }
        /// Initializer for Eigen types containers with explicit choice of transpose()
        template <typename Derived>
        void add(const Eigen::DenseBase<Derived> & var_to_be_logged, const std::string & name, const bool transpose) {
          auto container_ptr = std::make_shared<EigenContainer<Derived>>(var_to_be_logged, loggingDirectory, name, transpose);
          list.push_back(container_ptr);
        }

        /// Initializer for generic scalar types
        template <typename Derived>
        void add(const Derived & var_to_be_logged, const std::string & name) {
          auto container_ptr = std::make_shared<ScalarContainer<Derived>>(var_to_be_logged, loggingDirectory, name);
          list.push_back(container_ptr);
        }

        /// Log all variables from the list
        void logAll() {
          for(auto & logger: list) logger->log();
        }
    private:
        // Contains all the different type containers
        std::vector<std::shared_ptr<ContainerBase>> list;
        const std::string loggingDirectory;
    };


    template<typename M>
    M load_csv (const std::string & path) {
      std::ifstream indata;
      indata.open(path);
      std::string line;
      std::vector<double> values;
      uint rows = 0;
      while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
          values.push_back(std::stod(cell));
        }
        ++rows;
      }
      return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
    }

}
