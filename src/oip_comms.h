#ifndef OIP_COMMS_H
#define OIP_COMMS_H

#include "libplctag.h"
#include "open62541.h"
#include "S7Com.hpp"

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/thread.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <map>
#include <string>
#include <vector>
#include <queue>

#include "oip_blocking_queue.h"

struct AdsTagGroupImpl;
struct RtdeTagGroupImpl;
struct MqttTagGroupImpl;

namespace godot {

class OIPComms : public Node {
	GDCLASS(OIPComms, Node)

private:
	int timeout = 15000;
	double startup_timer = 0.0f;
	double register_wait_time = 500.0f;
	bool comms_error = false;
	String last_error = "";

	// Forward declaration so PlcTag can reference it.
	struct ArrayBuffer;

	struct PlcTag {
		bool initialized = false;
		int32_t tag_pointer = -1;
		int elem_count;

		// tag becomes dirty when a write happens before the next polled read
		// TBD - in the future expose an API so that "immediate reads" can occur
		// a little tricky with the current blocking queue/thread implementation
		bool dirty;

		// "One-Big-Tag" manifest routing. When a tag's name is found in
		// OIP_READ_NAMES / OIP_WRITE_NAMES / OIP_READ_DINT_NAMES /
		// OIP_WRITE_DINT_NAMES on the controller, discover_array_manifest
		// points read_buffer/write_buffer at the corresponding REAL[N] or
		// DINT[N] data tag and stores the slot index. The hot path then
		// dispatches that read/write directly into the buffer's libplctag
		// cache, skipping the per-tag libplctag handle entirely.
		ArrayBuffer *read_buffer = nullptr;
		int32_t read_slot = 0;
		ArrayBuffer *write_buffer = nullptr;
		int32_t write_slot = 0;
	};

	struct OpcUaTag {
		bool initialized = false;
		UA_NodeId node_id;
		UA_Variant value;
	};

	struct TagGroup {
		int polling_interval;
		double time;
		size_t init_count;
		bool init_count_emitted;
		bool has_error = false;

		String protocol;

		// gateway is a multi-purpose field:
		//   PLC:    IP address of the PLC, e.g. "192.168.1.200"
		//   OPC UA: server endpoint URL, e.g. "opc.tcp://192.168.56.104:62541"
		//   ADS:    remote PLC IPv4 address, e.g. "192.168.1.10"
		String gateway;

		// path:
		//   PLC:    rack/slot number, e.g. "1,2"
		//   OPC UA: unused (namespace is part of the tag name, e.g. "ns=2;i=5")
		//   ADS:    remote AmsNetId, e.g. "5.34.142.165.1.1"
		String path;

		// cpu:
		//   PLC:    CPU type string passed to libplctag
		//   ADS:    AMS port as a decimal string (e.g. "851" for the PLC runtime)
		String cpu;
		std::map<String, PlcTag> plc_tags;

		UA_Client *client;
		std::map<String, OpcUaTag> opc_ua_tags;

		// Defined in oip_comms.cpp; AdsLib pulls winsock2.h, which can't reach this header.
		AdsTagGroupImpl *ads_impl;

		RtdeTagGroupImpl *rtde_impl;

		MqttTagGroupImpl *mqtt_impl;

		bool needs_session_recovered_signal = false;

		// Per-group PLC request->response latency stats (microseconds).
		// Read stats wrap each `plc_tag_read` call in process_plc_read;
		// write stats wrap `plc_tag_write` in process_write. These measure
		// the actual libplctag round-trip for legacy-path tags (manifest
		// hot-path writes bypass the queue via auto_sync_write_ms and are
		// not captured here -- libplctag drives those async on its own
		// thread so the request->response time is not observable from
		// this side).
		//
		// Worker thread writes these fields; main thread reads them via
		// `get_{read,write}_latency_*_us` and resets them in
		// `set_sim_running(true)`. Plain uint64_t (no mutex/atomic) to
		// match the data-race-loose pattern used elsewhere in this class:
		// 8-byte-aligned uint64_t reads/writes are non-tearing on x86 and
		// monitoring stats tolerate transient inconsistency between
		// min/max/count display values.
		//
		// Placed at the end so the aggregate initializer in
		// `register_tag_group` doesn't need updating (default member
		// initializers fill these).
		uint64_t read_min_us = UINT64_MAX;
		uint64_t read_max_us = 0;
		uint64_t read_count = 0;
		uint64_t write_min_us = UINT64_MAX;
		uint64_t write_max_us = 0;
		uint64_t write_count = 0;

		// Write-queue delay (microseconds): time between the GDScript
		// caller invoking write_##a and the worker thread actually
		// starting process_write. Surfaces "Godot called write_bit, but
		// the worker was busy doing reads / other writes."
		uint64_t write_queue_min_us = UINT64_MAX;
		uint64_t write_queue_max_us = 0;
		uint64_t write_queue_count = 0;

		// Poll-cycle duration (microseconds): wall time of a single
		// process_tag_group dispatch -- i.e. the time to sweep every tag
		// in this group on the wire. The interesting number for "how
		// stale can Godot data be?" since fresh data is bounded by this
		// + the worker's queue gap.
		uint64_t poll_cycle_min_us = UINT64_MAX;
		uint64_t poll_cycle_max_us = 0;
		uint64_t poll_cycle_count = 0;
	};
	std::map<String, TagGroup> tag_groups;
	std::vector<String> tag_group_order;

	// "One-Big-Tag" manifest. PLC programmer publishes two controller-scope
	// REAL[N] buffers + parallel STRING[N] manifests per group:
	//
	//   OIP_READ_NAMES   STRING[N_R]   one tag name per slot
	//   OIP_READ         REAL[N_R]     PLC ladder COP/MOVs into here every scan
	//   OIP_WRITE_NAMES  STRING[N_W]   one tag name per slot
	//   OIP_WRITE        REAL[N_W]     ladder COP/MOVs from here every scan
	//
	// All-REAL routing: integer-typed tags (DINT/INT/SINT in the source
	// data) share the REAL buffer with floats and bools. Logix5000's
	// implicit REAL<->DINT coercion on assignment preserves values within
	// +/-2^24 (16.7M) -- safe for fault codes (<=0x00FFFFFF) and bitmasks.
	//
	// On sim start, discover_array_manifest reads the NAMES arrays, builds a
	// per-group slot map, creates ONE libplctag handle per data buffer
	// (2 buffers per group, not N), and attaches each registered PlcTag to
	// its slot. Wire traffic collapses from N per-tag CIP reads/sweep to
	// 2 array reads/sweep regardless of N.
	struct ArrayBuffer {
		int32_t libplctag_handle = -1;
		int elem_count = 0;
		bool is_write_direction = false; // false = OIP_READ, true = OIP_WRITE
		bool initialized = false;        // read buffer: first OK; write: at create
	};
	// Keyed by "<group_name>/<buffer_name>" with buffer_name in
	// {OIP_READ, OIP_READ_DINT, OIP_WRITE, OIP_WRITE_DINT}. Pointers cached
	// in PlcTag.read_buffer / write_buffer stay valid because std::map
	// insertions don't invalidate pointers and we only erase on cleanup.
	std::map<String, ArrayBuffer> array_buffers_;

	// Per-group discovery state. Reset on sim restart so the manifest is
	// re-read if the programmer changed it between runs.
	enum class ManifestState : uint8_t { NotStarted, Done, Failed };
	std::map<String, ManifestState> manifest_state_;

	// Cached manifest bindings, keyed by group_name -> tag_name. Populated
	// by discover_array_manifest from the OIP_*_NAMES STRING arrays. Used
	// by register_tag to wire late-registered PlcTags to their slot --
	// LCI typically calls register_tag AFTER set_sim_running(true), so the
	// existing-plc_tags walk in discover misses them and we need a lookup
	// the registration path can consult.
	struct ManifestBinding {
		ArrayBuffer *read_buffer = nullptr;
		int32_t read_slot = 0;
		ArrayBuffer *write_buffer = nullptr;
		int32_t write_slot = 0;
	};
	std::map<String, std::map<String, ManifestBinding>> manifest_bindings_;

	void discover_array_manifest(const String &group_name);

	// Hot-path dispatch helpers. The OIP_READ_FUNC / OIP_WRITE_FUNC macros
	// call into these when a PlcTag has read_buffer / write_buffer set.
	// One buffer slot is sizeof(REAL) = 4 bytes. (T)float reading != 0
	// gives the right truthiness for bool, so no specialisation is needed.
	template <typename T>
	static T array_buffer_read(const ArrayBuffer &ab, int32_t slot) {
		return (T)plc_tag_get_float32(ab.libplctag_handle, slot * 4);
	}
	template <typename T>
	static void array_buffer_write(ArrayBuffer &ab, int32_t slot, T value) {
		const int byte_off = slot * 4;
		const float newv = (float)value;
		const float cached = plc_tag_get_float32(ab.libplctag_handle, byte_off);
		if (cached == newv) return; // dedup; auto_sync_write_ms flushes on change
		plc_tag_set_float32(ab.libplctag_handle, byte_off, newv);
	}

	struct WriteRequest {
		uint8_t instruction;
		String tag_group_name;
		String tag_name;
		Variant value;
		// Time::get_ticks_usec() at the moment the GDScript caller invoked
		// write_##a. Subtracted from the worker's process_write start to
		// measure how long the request sat in the queue waiting for the
		// worker to pick it up. 0 if not set (legacy callers).
		uint64_t queued_us = 0;
	};
	std::queue<WriteRequest> write_queue;

	Ref<Thread> work_thread;
	bool work_thread_running = true;

	Ref<Thread> watchdog_thread;
	bool watchdog_thread_running = true;

	OIPBlockingQueue tag_group_queue;

	UA_Client *browse_client = nullptr;
	String browse_endpoint;

	bool browse_is_session_alive();
	bool ensure_browse_session();
	bool browse_reconnect();

	uint64_t last_ticks = 0;

	bool scene_signals_set = false;

	bool enable_comms = true;
	bool sim_running = false;

	bool enable_log = false;

	void watchdog();
	void process_work();

	void process_tag_group(const String &tag_group_name);
	void process_plc_tag_group(const String &tag_group_name);
	void process_opc_ua_tag_group(const String &tag_group_name);
	void process_ads_tag_group(const String &tag_group_name);
	void process_rtde_tag_group(const String &tag_group_name);
	void process_mqtt_tag_group(const String &tag_group_name);

	bool init_plc_tag(const String &tag_group_name, const String &tag_name);

	bool init_opc_ua_client(const String &tag_group_name);
	bool init_opc_ua_tag(const String &tag_group_name, const String &tag_path);

	bool init_ads_device(const String &tag_group_name);
	bool init_ads_tag(const String &tag_group_name, const String &tag_name);

	bool init_rtde_session(const String &tag_group_name);

	bool init_mqtt_client(const String &tag_group_name);
	bool init_mqtt_tag(const String &tag_group_name, const String &tag_name);

	bool opc_ua_client_connected(const String &tag_group_name);
	void invalidate_opc_ua_session(const String &tag_group_name, UA_StatusCode reason);
	bool ads_device_connected(const String &tag_group_name);
	bool rtde_session_started(const String &tag_group_name);
	bool mqtt_client_connected(const String &tag_group_name);
	bool tag_group_exists(const String &tag_group_name);
	bool tag_exists(const String &tag_group_name, const String &tag_name);

	void queue_tag_group(const String &tag_group_name);

	void flush_all_writes();
	void flush_one_write();

	void process_write(const WriteRequest &write_req);

	// process individual PLC read
	bool process_plc_read(const String &tag_group_name, PlcTag &tag, const String &tag_name);

	void opc_write(const String &tag_group_name, const String &tag_path);

#define OIP_DECLARE_OPC_SET(a)void opc_tag_set_##a(const String &tag_group_name, const String &tag_path, const godot::Variant value);

	OIP_DECLARE_OPC_SET(bit)
	OIP_DECLARE_OPC_SET(uint64)
	OIP_DECLARE_OPC_SET(int64)
	OIP_DECLARE_OPC_SET(uint32)
	OIP_DECLARE_OPC_SET(int32)
	OIP_DECLARE_OPC_SET(uint16)
	OIP_DECLARE_OPC_SET(int16)
	OIP_DECLARE_OPC_SET(uint8)
	OIP_DECLARE_OPC_SET(int8)
	OIP_DECLARE_OPC_SET(float64)
	OIP_DECLARE_OPC_SET(float32)

#define OIP_DECLARE_ADS_SET(a)void ads_tag_set_##a(const String &tag_group_name, const String &tag_path, const godot::Variant value);

	OIP_DECLARE_ADS_SET(bit)
	OIP_DECLARE_ADS_SET(uint64)
	OIP_DECLARE_ADS_SET(int64)
	OIP_DECLARE_ADS_SET(uint32)
	OIP_DECLARE_ADS_SET(int32)
	OIP_DECLARE_ADS_SET(uint16)
	OIP_DECLARE_ADS_SET(int16)
	OIP_DECLARE_ADS_SET(uint8)
	OIP_DECLARE_ADS_SET(int8)
	OIP_DECLARE_ADS_SET(float64)
	OIP_DECLARE_ADS_SET(float32)

	void rtde_tag_set(const String &tag_group_name, const String &tag_path, const godot::Variant value);

#define OIP_DECLARE_MQTT_SET(a)void mqtt_tag_set_##a(const String &tag_group_name, const String &tag_path, const godot::Variant value);

	OIP_DECLARE_MQTT_SET(bit)
	OIP_DECLARE_MQTT_SET(uint64)
	OIP_DECLARE_MQTT_SET(int64)
	OIP_DECLARE_MQTT_SET(uint32)
	OIP_DECLARE_MQTT_SET(int32)
	OIP_DECLARE_MQTT_SET(uint16)
	OIP_DECLARE_MQTT_SET(int16)
	OIP_DECLARE_MQTT_SET(uint8)
	OIP_DECLARE_MQTT_SET(int8)
	OIP_DECLARE_MQTT_SET(float64)
	OIP_DECLARE_MQTT_SET(float32)

	void cleanup_tag_groups();
	void cleanup_tag_group(const String &tag_group_name);

	void print(const Variant &message, bool error = false);

protected:
	static void _bind_methods();

public:
	enum TagType {
		TAG_TYPE_BOOL,
		TAG_TYPE_INT8,
		TAG_TYPE_UINT8,
		TAG_TYPE_INT16,
		TAG_TYPE_UINT16,
		TAG_TYPE_INT32,
		TAG_TYPE_UINT32,
		TAG_TYPE_INT64,
		TAG_TYPE_UINT64,
		TAG_TYPE_FLOAT32,
		TAG_TYPE_FLOAT64,
	};

	void register_tag_group(const String p_tag_group_name, const int p_polling_interval, const String p_protocol, const String p_gateway, const String p_path, const String p_cpu);
	bool register_tag(const String p_tag_group_name, const String p_tag_name, const int p_data_type);

	bool get_enable_comms();
	void set_enable_comms(bool value);

	bool get_sim_running();
	void set_sim_running(bool value);

	bool get_enable_log();
	void set_enable_log(bool value);

	String get_comms_error();

	Array get_tag_groups();

	// Per-group PLC request->response latency stats (microseconds).
	// Read stats wrap each `plc_tag_read`; write stats wrap each
	// `plc_tag_write`. count is the number of sampled calls. Reset on sim
	// start. Note: returns 0 for a group with no samples yet (min would
	// otherwise be UINT64_MAX).
	int get_read_latency_min_us(const String p_tag_group_name);
	int get_read_latency_max_us(const String p_tag_group_name);
	int get_read_latency_count(const String p_tag_group_name);
	int get_write_latency_min_us(const String p_tag_group_name);
	int get_write_latency_max_us(const String p_tag_group_name);
	int get_write_latency_count(const String p_tag_group_name);

	// Write-queue delay (GDScript-call -> worker-start). Returns
	// microseconds. 0 if no samples yet.
	int get_write_queue_min_us(const String p_tag_group_name);
	int get_write_queue_max_us(const String p_tag_group_name);
	int get_write_queue_count(const String p_tag_group_name);

	// Poll-cycle duration (full process_tag_group sweep). Returns
	// microseconds. 0 if no samples yet.
	int get_poll_cycle_min_us(const String p_tag_group_name);
	int get_poll_cycle_max_us(const String p_tag_group_name);
	int get_poll_cycle_count(const String p_tag_group_name);

#define OIP_DECLARE_FUNC(a, b)                                          \
	b read_##a(const String p_tag_group_name, const String p_tag_name); \
	void write_##a(const String p_tag_group_name, const String p_tag_name, const b p_value);

	OIP_DECLARE_FUNC(bit, bool)
	OIP_DECLARE_FUNC(uint64, uint64_t)
	OIP_DECLARE_FUNC(int64, int64_t)
	OIP_DECLARE_FUNC(uint32, uint32_t)
	OIP_DECLARE_FUNC(int32, int32_t)
	OIP_DECLARE_FUNC(uint16, uint16_t)
	OIP_DECLARE_FUNC(int16, int16_t)
	OIP_DECLARE_FUNC(uint8, uint8_t)
	OIP_DECLARE_FUNC(int8, int8_t)
	OIP_DECLARE_FUNC(float64, double)
	OIP_DECLARE_FUNC(float32, float)

	void clear_tag_groups();

	bool browse_connect(const String p_endpoint);
	void browse_disconnect();
	Array browse_children(const String p_node_id);
	Dictionary browse_node_info(const String p_node_id);

	void process();

	OIPComms();
	~OIPComms();
};

} //namespace godot

VARIANT_ENUM_CAST(godot::OIPComms::TagType);

#endif
