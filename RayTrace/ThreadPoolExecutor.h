#pragma once
#include <future>
#include <thread>
#include <vector>

#include "LinkedBlockQueue.h"

class ThreadPoolExecutor
{
public:
	ThreadPoolExecutor(int thread_cout): loop(true)
	{
		for (int i = 0; i < thread_cout; i++)
		{
			threads.emplace_back([this]()
				{
					while (loop)
						tasks.Get()(); 
				});
		}
	}
	~ThreadPoolExecutor() { shutdown(); }
public:
	template<typename F, typename ...Args>
	auto submit(F&& f, Args&& ...args) -> std::future<decltype(f(args...))>
	{
		using Ret = decltype(f(args...));
		std::shared_ptr<std::packaged_task<Ret()>> task = std::make_shared<std::packaged_task<Ret()>>(
			std::bind(std::forward<F>(f), std::forward<Args>(args)...));
		auto future = task->get_future();
		tasks.Put([task]() {
			task->operator()();
			});
		return future;
	}

	void shutdown() 
	{ 
		for (auto& t : threads)
			submit([this]() {loop = false; });

		for (auto& t : threads)
		{
			if (t.joinable())
				t.join();
		}
	}
private:
	LinkedBlockQueue<std::function<void()>> tasks;
	std::vector<std::thread> threads;
	bool loop;
};
