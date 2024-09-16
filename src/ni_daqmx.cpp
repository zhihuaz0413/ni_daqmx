#include "ni_daqmx.hpp"
#include <unistd.h> // for usleep()

namespace NI
{

    DAQcard::DAQcard(int n, int verbose) : m_n(n), ESC::CLI(verbose, "NI_DAQ")
    {
        logln("Initializing DAQ card with " + std::to_string(m_n) + " channels",
              true);
        std::string task_name = "Task";
        std::string ch_path = "Dev1/ai";
        std::string ch_name = "Channel";
        m_err = DAQmxCreateTask(task_name.c_str(), &m_taskHandle);
        check_error(m_err);
        for (int i = 0; i < m_n; i++)
        {
            m_err = DAQmxCreateAIVoltageChan(
                m_taskHandle, (ch_path + std::to_string(i)).c_str(),
                (ch_name + std::to_string(i)).c_str(), DAQmx_Val_Cfg_Default, -10.0,
                10.0, DAQmx_Val_Volts, NULL);
            check_error(m_err);
            logln(ch_name + std::to_string(i) + " created (" + ch_path +
                  std::to_string(i) + ")");
        }
        m_err = DAQmxStartTask(m_taskHandle);
        check_error(m_err);
    };
    DAQcard::~DAQcard() {};

    void DAQcard::read(float64 *data, uint64_t *timestamp_sec, uint64_t *timestamp_nsec)
    {
        m_err = DAQmxReadAnalogF64(m_taskHandle, m_samplingSize, 10.0,
                                   DAQmx_Val_GroupByChannel, data, m_n, &m_nb_read,
                                   NULL);
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        if (timestamp_sec != nullptr)
            *timestamp_sec = ts.tv_sec;
        if (timestamp_nsec != nullptr)
            *timestamp_nsec = ts.tv_nsec;

        check_error(m_err);
    };

    void DAQcard::check_error(int error)
    {
        if (DAQmxFailed(error))
        {
            DAQmxGetExtendedErrorInfo(m_errBuff, 2048);
            if (m_taskHandle != 0)
            {
                DAQmxStopTask(m_taskHandle);
                DAQmxClearTask(m_taskHandle);
            }
            if (DAQmxFailed(error))
                logln(ESC::fstr("Error", {ESC::FG_RED}) +
                          ESC::fstr(m_errBuff, {ESC::BOLD}),
                      true);
            logln("End of program, press Enter key to quit\n");
            exit(0);
        }
    }

    namespace ATI
    {

        void FT6_sensor::start_thread()
        {
            m_active = true;
            m_thread = new std::thread(&FT6_sensor::loop, this);
            logln("Thread started", true);
        };

        FT6_sensor::~FT6_sensor()
        {
            m_active = false;
            if (m_thread != nullptr)
            {
                m_thread->join();
                delete m_thread;
                delete m_mutex;
            }
        };

        void FT6_sensor::pause_thread()
        {
            m_active = false;
            logln("Thread paused", true);
        };

        double FT6_sensor::get_FT(unsigned i,
                                  uint64_t *timestamp_sec,
                                  uint64_t *timestamp_nsec) // Tz,Ty,Tx,Fz,Fy,Fx
        {
            std::lock_guard<std::mutex> lck(*m_mutex);
            i = (i < 6) ? i : 6;
            m_new_data[i] = false;
            if (timestamp_sec != nullptr)
                *timestamp_sec = m_timestamp_sec;
            if (timestamp_nsec != nullptr)
                *timestamp_nsec = m_timestamp_nsec;
            return m_data_conv[i];
        }

        void *FT6_sensor::loop(void *obj)
        {
            while (reinterpret_cast<FT6_sensor *>(obj)->m_active)
                reinterpret_cast<FT6_sensor *>(obj)->read_FT();
            return nullptr;
        };

        void FT6_sensor::read_FT()
        {
            uint64_t timestamp_sec, timestamp_nsec;
            m_card.read(m_data, &timestamp_sec, &timestamp_nsec);
            std::lock_guard<std::mutex> lck(*m_mutex);
            m_timestamp_sec = timestamp_sec;
            m_timestamp_nsec = timestamp_nsec;
            this->convert();
            if (m_callback != nullptr)
                m_callback(m_nb_channels, m_samplingSize, m_data_conv, m_timestamp_sec,
                           m_timestamp_nsec, m_callback_obj);
        };

        void FT6_sensor::convert()
        {
            for (int i = 0; i < 6; i++)
            {
                m_data_conv[i] = 0;
                for (int j = 0; j < 6; j++)
                    m_data_conv[i] += m_data[j] * m_calcoef[i][j];
                m_new_data[i] = true;
            }
        };
    } // namespace ATI
} // namespace NI
