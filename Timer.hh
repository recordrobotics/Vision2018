#include <chrono>

typedef unsigned long long int ms_t;

class Timer
{
public:
	static void init()
	{
		start = std::chrono::steady_clock::now();
	}

	static ms_t getMs()
	{
		ms_t d = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();

		return d;
	}

private:
	Timer() { }

	static std::chrono::time_point<std::chrono::steady_clock> start;
};
