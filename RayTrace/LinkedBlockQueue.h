#pragma once
#include <mutex>
#include <list>
#include <condition_variable>

template<typename T>
class LinkedBlockQueue
{
public:
	LinkedBlockQueue() {}
	~LinkedBlockQueue() {}

	void Put(const T& t);
	T Get();
	bool Get(T& t);
	size_t size() const;
private:
	std::mutex lock;
	std::list<T> list;
	std::condition_variable cv_get;
	std::condition_variable cv_put;
};

template<typename T>
inline void LinkedBlockQueue<T>::Put(const T& t)
{
	std::unique_lock<std::mutex> lock_(lock);
	while (list.max_size() == list.size())
		cv_put.wait(lock_);

	list.push_back(t);
	cv_get.notify_all();
}

template<typename T>
inline T LinkedBlockQueue<T>::Get()
{
	std::unique_lock<std::mutex> lock_(lock);
	while (list.empty())
		cv_get.wait(lock_);

	T t = std::move(list.front());
	list.pop_front();
	cv_put.notify_all();
	return t;
}

template<typename T>
inline bool LinkedBlockQueue<T>::Get(T& t)
{
	std::unique_lock<std::mutex> lock_(lock);
	if (list.empty())
		return false;

	t = list.front();
	list.pop_front();
	cv_put.notify_all();
	return true;
}

template<typename T>
inline size_t LinkedBlockQueue<T>::size() const
{
	std::unique_lock<std::mutex> lock_(lock);
	return list.size();
}
