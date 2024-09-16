#ifndef __NI_DAQMX_HPP
#define __NI_DAQMX_HPP

#include <NIDAQmx.h>
#include <iostream>
#include <stdio.h>  // printf
#include <stdlib.h> // malloc, exit, free
#include <string>
#include <time.h>

#include <atomic>
#include <mutex>
#include <thread>

#include "strANSIseq.hpp"

// nidaqmxconfig --self-test
// nidaqmxconfig --self-test /dev/nipalk
// g++ main.cpp /usr/lib/x86_64-linux-gnu/libnidaqmx.so

namespace NI
{

/**
 * @brief The NI_DAQMX class
 *
 * This class is a wrapper for the NI DAQmx library.
 * It provides a simple interface to the DAQmx library.
 *
 * @author Alexis Devillard
 */
class DAQcard : virtual public ::ESC::CLI
{
    public:
    /**
     * @brief Construct a new DAQcard object
     * @param n The number of channels to read on the card.
     */
    DAQcard(int n, int verbose = -1);
    ~DAQcard();
    void read(float64 *data,
              uint64_t *timestamp_sec = nullptr,
              uint64_t *timestamp_nsec = nullptr);
    void set_samplingSize(int nb_samples) { m_samplingSize = nb_samples; }

    private:
    void check_error(int error);

    int32 m_err = 0;
    TaskHandle m_taskHandle = 0;
    int32 m_nb_read;
    char m_errBuff[2048] = {'\0'};
    int m_n;
    int m_samplingSize = 1;
};

namespace ATI
{

/**
 * @brief The ATI 6axis force & torque sensor class
 *
 * It provides a simple interface to the ATI F/T sensor.
 *
 * @author Alexis Devillard
 */
class FT6_sensor : virtual public ESC::CLI
{
    public:
    FT6_sensor(int verbose = -1)
        : m_card(6, verbose - 1), ESC::CLI(verbose, "FT6_sensor")
    {
        m_mutex = new std::mutex();
    };
    ~FT6_sensor();

    void start_thread();
    void pause_thread();

    /**
     * @brief Returns the force or torque sensor data
     *
     * @param i Index of the sensor (0-5). Fx:0, Fy:1, Fz:2, Tx:3, Ty:4, Tz:5.
     * @return double The value of the sensor, in N or Nm
     */
    double get_FT(unsigned i,
                  uint64_t *timestamp_sec = nullptr,
                  uint64_t *timestamp_nsec = nullptr);

    //Tz,Ty,Tx
    double get_torque(unsigned i,
                      uint64_t *timestamp_sec = nullptr,
                      uint64_t *timestamp_nsec = nullptr)
    {
        return get_FT(i + 3, timestamp_sec, timestamp_nsec);
    }
    //Fz,Fy,Fx
    double get_force(unsigned i,
                     uint64_t *timestamp_sec = nullptr,
                     uint64_t *timestamp_nsec = nullptr)
    {
        return get_FT(i, timestamp_sec, timestamp_nsec);
    }

    double get_Fx(uint64_t *timestamp_sec = nullptr, uint64_t *timestamp_nsec = nullptr)
    {
        return get_FT(0, timestamp_sec, timestamp_nsec);
    };
    double get_Fy(uint64_t *timestamp_sec = nullptr, uint64_t *timestamp_nsec = nullptr)
    {
        return get_FT(1, timestamp_sec, timestamp_nsec);
    };
    double get_Fz(uint64_t *timestamp_sec = nullptr, uint64_t *timestamp_nsec = nullptr)
    {
        return get_FT(2, timestamp_sec, timestamp_nsec);
    };
    double get_Tx(uint64_t *timestamp_sec = nullptr, uint64_t *timestamp_nsec = nullptr)
    {
        return get_FT(3, timestamp_sec, timestamp_nsec);
    };
    double get_Ty(uint64_t *timestamp_sec = nullptr, uint64_t *timestamp_nsec = nullptr)
    {
        return get_FT(4, timestamp_sec, timestamp_nsec);
    };
    double get_Tz(uint64_t *timestamp_sec = nullptr, uint64_t *timestamp_nsec = nullptr)
    {
        return get_FT(5, timestamp_sec, timestamp_nsec);
    };

    void set_callback(void (*callback)(int nb_channels,
                                       int nb_samples,
                                       double *data,
                                       uint64_t timestamp_sec,
                                       uint64_t timestamp_nsec,
                                       void *obj),
                      void *obj)
    {
        m_callback = callback;
        m_callback_obj = obj;
    }

    /**
     * @brief Returns true if new data are available.
     *
     * @param i Selects the channel to check (0-5). Fx:0, Fy:1, Fz:2, Tx:3, Ty:4, Tz:5. Channel are updated consecutively so testing the first one should be enough
     * @return true If new data is available.
     * @return false If no new data is available.
     */
    bool has_new_data(int i = 0) { return m_new_data[i]; };

    static void *loop(void *obj);

    /**
     * @brief Reads the NI::DAQcard and updates the data. Needs to be called manually if the thread is not active.
     *
     */
    void read_FT();

    private:
    void convert();

    std::thread *m_thread = nullptr;
    std::mutex *m_mutex;
    bool m_active = false;

    void (*m_callback)(int nb_channels,
                       int nb_samples,
                       double *data,
                       uint64_t timestamp_sec,
                       uint64_t timestamp_nsec,
                       void *obj) = nullptr;
    void *m_callback_obj = nullptr;
    int m_nb_channels = 6;
    int m_samplingSize = 1;

    NI::DAQcard m_card;
    double m_data[6];
    double m_data_conv[6];
    uint64_t m_timestamp_sec = 0;
    uint64_t m_timestamp_nsec = 0;
    bool m_new_data[6] = {false};
    float m_calcoef[6][6] = {
        {-0.00758, -0.00595, 0.17292, -3.31572, -0.10515, 3.04384},
        {-0.06741, 3.73876, 0.06286, -1.92243, 0.06722, -1.73453},
        {3.59265, 0.12395, 3.72948, 0.10916, 3.47531, 0.17706},
        {-0.37381, 22.57870, 21.6181, -10.8513, -18.7227, -11.5072},
        {-23.5661, -0.97300, 10.7964, 20.7230, 11.9170, -11.9916},
        {-0.51999, 14.04470, -0.63699, 15.2440, -0.38282, 13.2183}};
};

}; // namespace ATI

} // namespace NI
#endif
