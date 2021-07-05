#include "database.h"
#include <math/common_math.h>
#include <math/math_utils.h>
#include <cmath>
#include <user_time/user_time.h>

#ifndef AP_OADATABASE_TIMEOUT_SECONDS_DEFAULT
    #define AP_OADATABASE_TIMEOUT_SECONDS_DEFAULT   10
#endif

#ifndef AP_OADATABASE_SIZE_DEFAULT
    #define AP_OADATABASE_SIZE_DEFAULT          100
#endif

#ifndef AP_OADATABASE_QUEUE_SIZE_DEFAULT
    #define AP_OADATABASE_QUEUE_SIZE_DEFAULT 80
#endif

#ifndef AP_OADATABASE_DISTANCE_FROM_HOME
    #define AP_OADATABASE_DISTANCE_FROM_HOME 3
#endif


AP_OADatabase::AP_OADatabase()
{
    if (_singleton != nullptr) {
        printf("AP_OADatabase must be singleton\n");
    }
    _singleton = this;
}


void AP_OADatabase::init(void)
{
    init_database();
    init_queue();

    // initialise scalar using beam width of at least 1deg
    dist_to_radius_scalar = tanf(math::radians(std::fmax(_beam_width, 1.0f)));

    if (!healthy()) {
        printf("DB init failed . Sizes queue:%u, db:%u", (unsigned int)_queue.size, (unsigned int)_database.size);
        delete _queue.items;
        delete[] _database.items;
        return;
    }
}

void AP_OADatabase::update()
{
    if (!healthy()) {
        return;
    }
    process_queue();
    database_items_remove_all_expired();
}

// push a location into the database
void AP_OADatabase::queue_push(const Vector3f &pos, uint32_t timestamp_ms, float distance)
{
    if (!healthy()) {
        return;
    }

    // ignore objects that are far away
    if ((_dist_max > 0.0f) && (distance > _dist_max)) {
        return;
    }

   const OA_DbItem item = {pos, timestamp_ms, std::fmax(_radius_min, distance * dist_to_radius_scalar), 0, AP_OADatabase::OA_DbItemImportance::Normal};
    {
        std::lock_guard<std::mutex> locker(_queue.sem);
        _queue.items->push(item);
    }
}

void AP_OADatabase::init_queue()
{
    _queue.size = _queue_size_param;
    if (_queue.size == 0) {
        return;
    }

    _queue.items = new ObjectBuffer<OA_DbItem>(_queue.size);
}

void AP_OADatabase::init_database()
{
    _database.size = _database_size_param;
    if (_database_size_param == 0) {
        return;
    }

    _database.items = new OA_DbItem[_database.size];
}

// get bitmask of gcs channels item should be sent to based on its importance
// returns 0xFF (send to all channels) if should be sent, 0 if it should not be sent
uint8_t AP_OADatabase::get_send_to_gcs_flags(const OA_DbItemImportance importance)
{
    switch (importance) {
    case OA_DbItemImportance::Low:
        if (_output_level >= (int8_t)OA_DbOutputLevel::OUTPUT_LEVEL_SEND_ALL) {
            return 0xFF;
        }
        break;

    case OA_DbItemImportance::Normal:
        if (_output_level >= (int8_t)OA_DbOutputLevel::OUTPUT_LEVEL_SEND_HIGH_AND_NORMAL) {
            return 0xFF;
        }
        break;

    case OA_DbItemImportance::High:
        if (_output_level >= (int8_t)OA_DbOutputLevel::OUTPUT_LEVEL_SEND_HIGH) {
            return 0xFF;
        }
        break;
    }
    return 0x0;
}

// returns true when there's more work inthe queue to do
bool AP_OADatabase::process_queue()
{
    if (!healthy()) {
        return false;
    }

    // processing queue by moving those entries into the database
    // Using a for with fixed size is better than while(!empty) because the
    // while could get us stuck here longer than expected if we're getting
    // a lot of values pushing into it while we're trying to empty it. With
    // the for we know we will exit at an expected time
    const uint16_t queue_available = std::fmin(_queue.items->available(), 100U);
    if (queue_available == 0) {
        return false;
    }

    for (uint16_t queue_index=0; queue_index<queue_available; queue_index++) {
        OA_DbItem item;

        bool pop_success;
        {
            std::lock_guard<std::mutex> locker(_queue.sem);
            pop_success = _queue.items->pop(item);
        }
        if (!pop_success) {
            return false;
        }

        item.send_to_gcs = get_send_to_gcs_flags(item.importance);

        // compare item to all items in database. If found a similar item, update the existing, else add it as a new one
        bool found = false;
        for (uint16_t i=0; i<_database.count; i++) {
            if (is_close_to_item_in_database(i, item)) {
                database_item_refresh(i, item.timestamp_ms, item.radius);
                found = true;
                break;
            }
        }

        if (!found) {
            database_item_add(item);
        }
    }
    return (_queue.items->available() > 0);
}

void AP_OADatabase::database_item_add(const OA_DbItem &item)
{
    if (_database.count >= _database.size) {
        return;
    }
    _database.items[_database.count] = item;
    _database.items[_database.count].send_to_gcs = get_send_to_gcs_flags(_database.items[_database.count].importance);
    _database.count++;
}

void AP_OADatabase::database_item_remove(const uint16_t index)
{
    if (index >= _database.count || _database.count == 0) {
        // index out of range
        return;
    }

    // radius of 0 tells the GCS we don't care about it any more (aka it expired)
    _database.items[index].radius = 0;
    _database.items[index].send_to_gcs = get_send_to_gcs_flags(_database.items[index].importance);

    _database.count--;
    if (_database.count == 0) {
        return;
    }

    if (index != _database.count) {
        // copy last object in array over expired object
        _database.items[index] = _database.items[_database.count];
        _database.items[index].send_to_gcs = get_send_to_gcs_flags(_database.items[index].importance);
    }
}

void AP_OADatabase::database_item_refresh(const uint16_t index, const uint32_t timestamp_ms, const float radius)
{
    if (index >= _database.count) {
        // index out of range
        return;
    }

    const bool is_different =
            (fabs(_database.items[index].radius - radius) > 1e-3) ||
            (timestamp_ms - _database.items[index].timestamp_ms >= 500);

    if (is_different) {
        // update timestamp and radius on close object so it stays around longer
        // and trigger resending to GCS
        _database.items[index].timestamp_ms = timestamp_ms;
        _database.items[index].radius = radius;
        _database.items[index].send_to_gcs = get_send_to_gcs_flags(_database.items[index].importance);
    }
}

void AP_OADatabase::database_items_remove_all_expired()
{
    // calculate age of all items in the _database

    if (_database_expiry_seconds <= 0) {
        // zero means never expire. This is not normal behavior but perhaps you could send a static
        // environment once that you don't want to have to constantly update
        return;
    }

    const uint32_t now_ms = user_time::get_millis();
    const uint32_t expiry_ms = (uint32_t)_database_expiry_seconds * 1000;
    uint16_t index = 0;
    while (index < _database.count) {
        if (now_ms - _database.items[index].timestamp_ms > expiry_ms) {
            database_item_remove(index);
        } else {
            index++;
        }
    }
}

// returns true if a similar object already exists in database. When true, the object timer is also reset
bool AP_OADatabase::is_close_to_item_in_database(const uint16_t index, const OA_DbItem &item) const
{
    if (index >= _database.count) {
        // index out of range
        return false;
    }

    const float distance_sq = (_database.items[index].pos - item.pos).length_squared();
    return ((distance_sq < math::Sqr(item.radius)) || (distance_sq < math::Sqr(_database.items[index].radius)));
}

// send ADSB_VEHICLE mavlink messages
void AP_OADatabase::send_adsb_vehicle(void)
{
    
}

// singleton instance
AP_OADatabase *AP_OADatabase::_singleton;

namespace AP {
AP_OADatabase *oadatabase()
{
    return AP_OADatabase::get_singleton();
}

}


