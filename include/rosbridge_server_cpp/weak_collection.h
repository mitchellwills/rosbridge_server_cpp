#ifndef ROSBRIDGE_SERVER_CPP_WEAK_COLLECTION_H
#define ROSBRIDGE_SERVER_CPP_WEAK_COLLECTION_H

#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/foreach.hpp>

namespace rosbridge_server_cpp {

template <typename T>
class WeakCollection {
public:
  typedef boost::shared_ptr<T> ItemPtr;
  typedef boost::weak_ptr<T> ItemWeakPtr;

  WeakCollection() : is_closed_(false) {}
  ~WeakCollection() {
    close();
  }

  ItemPtr add(T* new_item) {
    boost::unique_lock<boost::mutex> lock(mutex_);
    if(is_closed_) {
      delete new_item;
      return ItemPtr();
    }
    ItemPtr ptr(new_item, boost::bind(&WeakCollection::deleteItem, this, _1));
    items_[new_item] = ptr;
    return ptr;
  }

  void close() {
    std::vector<ItemWeakPtr> to_close;
    {
      boost::unique_lock<boost::mutex> lock(mutex_);
      if(is_closed_)
	return;
      is_closed_ = true;

      typedef typename std::map<T*, ItemWeakPtr>::value_type entry;
      BOOST_FOREACH(const entry& item, items_) {
	to_close.push_back(item.second);
      }
    }


    BOOST_FOREACH(const ItemWeakPtr& item, to_close) {
      ItemPtr item_lock = item.lock();
      if (item_lock)
	item_lock->close();
    }

    {
      boost::unique_lock<boost::mutex> lock(mutex_);
      while(items_.size() > 0) {
	empty_cv_.wait(lock);
      }
    }
  }

private:
  void deleteItem(T* item) {
    {
      boost::unique_lock<boost::mutex> lock(mutex_);
      items_.erase(item);
      // delete while holding the lock so that we do not report empty before fully cleaned up
      delete item;
    }
    empty_cv_.notify_all();
  }

  boost::condition_variable empty_cv_;
  boost::mutex mutex_;
  std::map<T*, ItemWeakPtr> items_;
  bool is_closed_;
};

}

#endif
