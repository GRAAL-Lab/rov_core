/**
 * @author Francesco Wanderlingh
 * @date Year 2018
 *
 * @brief A set of useful C++ utilities ironically called "futils".
 *
 * @details The utilities implemented, for Linux-based OS, include:\n
 * 			- Elapsing Spinner Command-Line Animation\n
 * 			- Waiting Dotter Command-Line Animation\n
 * 			- Executable file self path retrieval for path-safe file saving, loading\n
 * 			- STL Vector Printing\n
 * 			- Debug print macro\n
 * 			- UDP Sender/Receiver Sockets
 */

#ifndef FUTILS_H_
#define FUTILS_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <unistd.h>
#include <map>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <pwd.h>
#include <iomanip>
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <array>

#ifdef DEBUG_PRINT
#	define dout std::cerr
#	define d_out(x) (std::cerr << x << std::endl)
#else
#	define dout 0 && std::cerr
#	define d_out(x) 0 && std::cerr
#endif

/** Escape sequence
 *  \033[<code>m or \e[<code>m
 *
 *  color : \[\033[ <code>m\] \]
 *
 *  e.g.
 *   bg=Blue, Bold, fg=Red
 *    1. \033[44;1;31m
 *    2. \033[44m\033[1;31m
 *
 *
 * Text attributes
 *  0  All attributes off
 *  1  Bold on
 *  4  Underscore (on monochrome display adapter only)
 *  5  Blink on
 *  7  Reverse video on
 *  8  Concealed on
 *
 *
 * Foreground colors
 *  0;30   Black          1;30   Dark Gray
 *  0;31   Red            1;31   Light Red
 *  0;32   Green          1;32   Light Green
 *  0;33   Brown          1;33   Yellow
 *  0;34   Blue           1;34   Light Blue
 *  0;35   Purple         1;35   Light Purple
 *  0;36   Cyan           1;36   Light Cyan
 *  0;37   Light Gray     1;37   White
 */

#if defined(__linux__) || defined(linux)

namespace tc
{
const char* const none = "\033[0m";
const char* const black = "\033[0;30m";
const char* const grayD = "\033[1;30m";
const char* const red = "\033[0;31m";
const char* const redL = "\033[1;31m";
const char* const green = "\033[0;32m";
const char* const greenL = "\033[1;32m";
const char* const brown = "\033[0;33m";
const char* const yellow = "\033[1;33m";
const char* const blu = "\033[0;34m";
const char* const bluL = "\033[1;34m";
const char* const mag = "\033[0;35m";
const char* const magL = "\033[1;35m";
const char* const cyan = "\033[0;36m";
const char* const cyanL = "\033[1;36m";
const char* const grayL = "\033[0;37m";
const char* const white = "\033[1;37m";
}
#endif



namespace futils
{

template<class Object>
void PrettyPrint(const Object& A, std::string name, const char* color = tc::white)
{
    std::cout << color << name << tc::none << std::endl << A << std::endl;
}

/**
 * @brief Create folder if not existing
 *
 * @param path of the folder
 * @return 0 if success (or folder exists) -1 otherwise (sets errno)
 */
inline int MakeDir(const std::string path)
{
    /**
	 *  S_IRWXU | S_IRWXG | S_IRWXO
	 *  Read/write/search permissions for owner and group and others. Since mkdir() masks
	 *  the mode with umask(), a further chmod() is needed.
         */
    int ret;
    ret = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
    if (ret != 0) {
        if (errno != EEXIST) {
            std::cerr << tc::redL << "Could not create directory " << path << " (error: " << strerror(errno) << ")\n";
            return false;
        } else {
            return true;
        }
    }
    chmod(path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
    return true;
    //}
}

/// My first functor!!! :D

/**
 * Prints a spinner that, put in a while loop with negligible execution time,
 * spins at a given frequency.
 *
 * @param freq Animation update frequency
 */
struct Spinner
{
    Spinner(int frequency) :
        freq(frequency), spinIndex(0), spin_chars("/-\\|")
    {
        clock_gettime(CLOCK_MONOTONIC, &last);
        period = 1 / static_cast<double>(freq + 1E-6);
        //std::cout << "period: " << period << "s" << std::endl;
    }

    void operator()(void)
    {

        clock_gettime(CLOCK_MONOTONIC, &now);
        double timeElapsed = (now.tv_sec - last.tv_sec) + (now.tv_nsec - last.tv_nsec) / 1E9;

        //std::cout << "timeElapsed: " << timeElapsed << std::endl;
        if (period - timeElapsed < 1E-3) {
            //std::cout << "fabs(freq - timeElapsed): " << fabs(freq - timeElapsed) << std::endl;
            //printf("\e[?25l"); /* hide the cursor */
            putchar(' ');
            putchar(spin_chars[spinIndex % spin_chars.length()]);
            putchar(' ');
            fflush(stdout);
            putchar('\r');
            spinIndex++;
            last = now;
        }

    }

private:
    struct timespec last, now;
    int freq;
    double period;
    unsigned long spinIndex;
    std::string spin_chars;
};

/**
 * Prints a classic dot animation that, put in a while loop with negligible execution time,
 * plays at a given frequency.
 *
 * @param freq Animation update frequency
 */
struct Dotter
{
    Dotter(int freq) :
        freq(freq), spinIndex(0), spin_chars("... .. .. .. .... .... .")
    {
        clock_gettime(CLOCK_MONOTONIC, &last);
        period = 1 / static_cast<double>(freq);
        //std::cout << "period: " << period << "s" << std::endl;
    }

    void operator()(void)
    {

        clock_gettime(CLOCK_MONOTONIC, &now);
        double timeElapsed = (now.tv_sec - last.tv_sec) + (now.tv_nsec - last.tv_nsec) / 1E9;

        //std::cout << "timeElapsed: " << timeElapsed << std::endl;
        if (period - timeElapsed < 1E-3) {
            //std::cout << "fabs(freq - timeElapsed): " << fabs(freq - timeElapsed) << std::endl;
            //putchar(' ');
            putchar(spin_chars[spinIndex % spin_chars.length()]);
            putchar(spin_chars[(spinIndex % spin_chars.length()) + 1]);
            putchar(spin_chars[(spinIndex % spin_chars.length()) + 2]);
            putchar(spin_chars[(spinIndex % spin_chars.length()) + 3]);
            putchar(' ');
            fflush(stdout);
            putchar('\r');
            spinIndex += 4;
            last = now;
        }

    }

private:
    struct timespec last, now;
    int freq;
    double period;
    unsigned long spinIndex;
    std::string spin_chars;
};

/**
 * Simple timer for getting time elapsed and intermediate laps.
 * All values are returned in seconds, with nanosecond precision.
 */
struct Timer
{
    Timer() :
        running(false), elapsedTime(0), lapTime(0)
    {
        clock_gettime(CLOCK_MONOTONIC, &now);
        lap = now;
    }

    void Start()
    {
        clock_gettime(CLOCK_MONOTONIC, &start);
        lap = start;
        running = true;
    }

    void Stop()
    {
        lapTime = 0;
        running = false;
    }

    /**
	 * @return elapsed time since start in seconds, with nanosecond precision (if timer is running)
         */
    double Elapsed()
    {
        if (running) {
            clock_gettime(CLOCK_MONOTONIC, &now);
            elapsedTime = (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1E9;
            return elapsedTime;
        } else {
            return 0.0;
        }
    }

    /**
	 * @return elapsed time since last Lap in seconds, with nanosecond precision
         */
    double Lap()
    {
        clock_gettime(CLOCK_MONOTONIC, &now);
        lapTime = (now.tv_sec - lap.tv_sec) + (now.tv_nsec - lap.tv_nsec) / 1E9;
        laps.push_back(lapTime);
        lap = now;
        return lapTime;
    }

    double GetCurrentLapTime()
    {
        clock_gettime(CLOCK_MONOTONIC, &now);
        lapTime = (now.tv_sec - lap.tv_sec) + (now.tv_nsec - lap.tv_nsec) / 1E9;
        return lapTime;
    }

    std::vector<double> laps;
    bool running;

private:
    struct timespec start, lap, now;
    double elapsedTime, lapTime;
};

struct Percentage
{
    Percentage(int iterations) :
        numIterations(iterations), currentIteration(0), currentPerc(0.0)
    {
    }

    void operator()(void)
    {
        currentIteration++;
        currentPerc = ((double) currentIteration * 100.0 / (double) numIterations);
        std::cout << "\r" << std::setprecision(3) << "Progress: " << currentPerc << "%    ";
    }

private:
    int numIterations;
    int currentIteration;
    double currentPerc;
};

template<typename T>
inline std::string toStringPointDecimal(T val)
{
    std::string s = std::to_string(val);
    std::replace(s.begin(), s.end(), ',', '.');
    return s;
}

/*
 template<typename T>
 inline std::string toStringPointDecimal(T val,  const int precision = 0){
 std::streamsize defaultPrecision = std::cout.precision();
 if(precision != 0){
 std::setprecision(static_cast<std::streamsize>(precision));
 }
 std::ostringstream oss;
 oss << std::setprecision(precision) << val;
 std::string s = oss.str();
 std::replace(s.begin(), s.end(), ',', '.');
 std::setprecision(defaultPrecision);
 return s;
 }
 */
template<typename T>
void PrintArray(T arr, const int size, const char delimiter)
{
    for (int i = 0; i < size; ++i) {
        std::cout << arr[i];
        if (i < (size - 1)) {
            std::cout << delimiter << " ";
        }
    }
}

template<typename T>
void PrintCMATArray(T arr, const int size, const char delimiter)
{
    for (int i = 1; i <= size; ++i) {
        std::cout << arr(i);
        if (i < size) {
            std::cout << delimiter << " ";
        }
    }
}

template<typename T>
std::string ArrayToString(T arr, const int size, const char delimiter)//, const int precision = 0)
{
    std::string arrayText;
    for (int i = 0; i < size; ++i) {
        arrayText = arrayText + toStringPointDecimal(arr[i]);
        if (i < (size - 1)) {
            arrayText = arrayText + delimiter + " ";
        }
    }
    return arrayText;
}

template<typename T>
std::string CMATArrayToString(T arr, const int size, const char delimiter)
{
    std::string arrayText;
    for (int i = 1; i <= size; ++i) {
        arrayText = arrayText + toStringPointDecimal(arr(i));
        if (i < size) {
            arrayText = arrayText + delimiter + " ";
        }
    }
    return arrayText;
}

template<typename T>
void PrintSTLVector(T vecObj, const char delimiter, const std::string preText = "")
{
    std::cout << preText << " ";
    for (typename T::iterator itr = vecObj.begin(); itr != vecObj.end(); ++itr) {
        std::cout << *itr;
        if (itr != (vecObj.end() - 1)) {
            std::cout << delimiter << " ";
        }
    }
}

template<typename T>
std::string STLVectorToString(T vecObj, const char delimiter, const std::string preText = "")
{
    std::string vectorText;
    vectorText = preText + " ";
    for (typename T::iterator itr = vecObj.begin(); itr != vecObj.end(); ++itr) {
        vectorText = vectorText + toStringPointDecimal(*itr);
        if (itr != (vecObj.end() - 1)) {
            vectorText = vectorText + delimiter + " ";
        }
    }
    return vectorText;
}

template<typename T>
void PrintSTLVectOfVects(T vecObj, const char delimiter)
{
    for (typename T::iterator itr = vecObj.begin(); itr != vecObj.end(); ++itr) {
        std::cout << "#" << itr - vecObj.begin() << ": ";
        PrintSTLVector(*itr, delimiter);
        std::cout << "\n";
    }
}

/**
 * Returns the current date and time formatted as %Y-%m-%d_%H.%M.%S
 *
 * @return Current date
 */
inline std::string GetCurrentDateFormatted()
{
    std::time_t t = std::time(NULL);
    char mbstr[20];
    std::strftime(mbstr, sizeof(mbstr), "%Y-%m-%d_%H.%M.%S", std::localtime(&t));
    std::string currentDate(mbstr);

    return currentDate;
}

template<typename K, typename V>
using MapIterator = typename std::map<K,V>::const_iterator;

template<typename K, typename V>
bool FindMapKeyByValue(const std::map<K, V> myMap, const V value, K &key)
{
    bool keyWasFound = false;
    MapIterator<K, V> it;
    for (it = myMap.begin(); it != myMap.end(); ++it) {
        if (it->second == value) {
            key = it->first;
            keyWasFound = true;
            break;
        }
    }
    return keyWasFound;
}

/// Functor for getting sum of previous result and square of current element (inner product can be used instead)
template<typename T>
struct square
{
    T operator()(const T& Left, const T& Right) const
    {
        return (Left + Right * Right);
    }
};

#if defined(__linux__) || defined(linux)
/**
 * Returns the path of the folder containing executable that calls this functions
 *
 * @return String with the folder path
 */
inline std::string get_selfpath()
{
    char buff[2048];
    ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff) - 1);
    if (len != -1) {
        buff[len] = '\0';
        std::string path(buff);   ///Here the executable name is still in
        std::string::size_type t = path.find_last_of("/");   // Here we find the last "/"
        path = path.substr(0, t);                             // and remove the rest (exe name)
        return path;
    } else {
        printf("Cannot determine executable path! [Exiting]\n");
        exit(-1);
    }
}

/**
 * Return user "home" directory
 */
inline std::string get_homepath()
{
    const char *homedir;

    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }
    return homedir;

}

inline bool does_file_exists(const std::string& name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

inline std::string exec(const char* cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
        throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != NULL)
            result += buffer.data();
    }
    return result;
}

inline void ParseIPString(const std::string input_Str, unsigned char ip[4])
{
    std::string ipString = input_Str, ip_chunk;
    std::string::size_type t1;
    for (int i = 0; i < 3; ++i) {
        t1 = ipString.find_first_of(".");
        ip_chunk = ipString.substr(0, t1);
        ip[i] = static_cast<unsigned char>(stod(ip_chunk, &t1));
        ipString = ipString.substr(t1 + 1, ipString.size());
    }
    ip[3] = static_cast<unsigned char>(stod(ipString, &t1));
}

inline void paddr(unsigned char *a)
{
    printf("%d.%d.%d.%d\n", a[0], a[1], a[2], a[3]);
}

class UDPSenderSocket
{
public:
    UDPSenderSocket() :
        sockfd(0)
    {
        slen = sizeof(si_server);
    }

    UDPSenderSocket(const char ip[], const char port[]) :
        UDPSenderSocket()
    {
        Configure(ip, port);
    }


    void Configure(const char ip[], const char port[])
    {
        if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
            perror("UPDSenderSocket - socket creation failed");
            exit(1);
        }

        uint16_t serverPort = static_cast<uint16_t>(stod(std::string(port)));
        ConfigureSenderSocket(si_server, ip, serverPort);
    }

    template<typename TP>
    int Send(TP dataToSend, std::size_t dataSize)
    {
        return sendto(sockfd, dataToSend, dataSize, 0, (struct sockaddr *) &si_server, slen);

    }

private:
    bool ConfigureSenderSocket(struct sockaddr_in &si_out, const char *ip, uint16_t port)
    {

        // zero out the structure
        memset((char *) &si_out, 0, sizeof(si_out));
        si_out.sin_family = AF_INET;
        si_out.sin_port = htons(port);

        if (inet_aton(ip, &si_out.sin_addr) == 0) {
            fprintf(stderr, "inet_aton() failed\n");
            exit(1);
        }

        return true;
    }

    struct sockaddr_in si_server;
    int sockfd;
    socklen_t slen;
};

class UPDReceiverSocket
{
public:
    UPDReceiverSocket() :
        sockfd_(0)
    {
        slen_ = sizeof(si_client_);
    }

    UPDReceiverSocket(char port[], bool nonBlocking) :
        UPDReceiverSocket()
    {
        Configure(port, nonBlocking);
    }

    void Configure(char port[], bool nonBlocking)
    {
        uint16_t port_ = static_cast<uint16_t>(stod(std::string(port)));
        ConfigureReceiverSocket(sockfd_, si_client_, port_, nonBlocking);
    }

    template<typename TP>
    int Receive(TP data, std::size_t dataSize)
    {
        return recvfrom(sockfd_, data, dataSize, 0, (struct sockaddr *) &si_client_, &slen_);

    }

private:
    bool ConfigureReceiverSocket(int &sockfd, struct sockaddr_in &si_in, uint16_t port, bool nonBlocking)
    {
        ///
        /// SOCKET Initialization
        ///
        if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
            perror("socket");
            exit(1);
        }

        // zero out the structure
        memset((char *) &si_in, 0, sizeof(si_in));
        si_in.sin_family = AF_INET;
        si_in.sin_port = htons(port);
        si_in.sin_addr.s_addr = htonl(INADDR_ANY);

        //bind socket to port
        if (bind(sockfd, (struct sockaddr*) &si_in, sizeof(si_in)) == -1) {
            perror("bind error");
            exit(1);
        }

        if (nonBlocking) {
            // Put the socket in non-blocking mode:
            if (fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFL) | O_NONBLOCK) < 0) {
                perror("Error in putting the socket in non blocking mode");
                exit(1);
            }
        }

        return true;
    }

    struct sockaddr_in si_client_;
    int sockfd_;
    socklen_t slen_;			// = sizeof(si_tool);
};

#endif /* Linux functions*/

inline bool CopyFile(const std::string& inputFile, const std::string& outputFile)
{
    bool ret(false);

    if (does_file_exists(inputFile)) {
        std::ifstream src(inputFile, std::ios::binary);
        std::ofstream dst(outputFile, std::ios::binary);
        chmod(outputFile.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
        dst << src.rdbuf();
        dst.close();
        ret = true;
    } else {
        std::cout << "Error in futils::CopyFile(), file='" << inputFile << "' does not exist!" << std::endl;
    }

    return ret;
}

/**
	 * This function performs a simple 1D filtering over an array of data. The
	 * result is the output of the following function:
	 * y(i) = alpha * x(i) + ( 1 - alpha ) * y(i-1)
	 *
	 * alpha is the 'b' output parameter of the 'ellip' MATLAB function:
	 * [b, a] = ellip(filterOrder, passbandRipple, stopbandAtt, radPerSample)
	 *
	 * @param array
	 * @param size
	 * @param alpha
	 * @return
         */
struct FirstOrderFilter
{
    std::vector<double> prevState_;
    std::vector<double> filteredData_;
    bool isRunning_, initialised_;
    int dataSize_;
    double alpha_;

    FirstOrderFilter()
    {
        Reset();
    }

    void Reset()
    {
        isRunning_ = false;
    }

    void Init(const int dataSize, const double alpha)
    {
        dataSize_ = dataSize;
        prevState_.resize(dataSize);
        filteredData_.resize(dataSize);
        alpha_ = alpha;

        initialised_ = true;
    }

    std::vector<double>& Filter(const std::vector<double> array)
    {
        // If the filter is not yet running we don't have any previous state so
        // we use the current one as a pseudo-previous state.
        if (initialised_) {
            if (!isRunning_) {
                for (int i = 0; i < dataSize_; ++i) {
                    prevState_.at(i) = array.at(i);
                }
                isRunning_ = true;
            }

            for (int i = 0; i < dataSize_; ++i) {
                filteredData_.at(i) = alpha_ * array.at(i) + (1 - alpha_) * prevState_.at(i);
            }
            return filteredData_;
        } else {
            std::cout << "Error: FirstOrderFilter::Init(dataSize, alpha) was not called!\n" << "Exiting..."
                      << std::endl;
            exit(EXIT_FAILURE);
        }
    }
};

} /* namespace FUTILS */

#endif /* FUTILS_H_ */

