#include <Arduino.h>
#include <arduino_lmic_hal_boards.h>
#include <Wire.h>

// leuville-arduino-easy-lmic
#include <JsonEndnode.h>
// leuville-arduino-utilities
#include <misc-util.h>
// LoRaWAN configuration: see OTAAId in LMICWrapper.h
#include <lora-common-defs.h>

#include <StatusLed.h>
#include <ISRTimer.h>
#include <ISRWrapper.h>
#include <JobRegister.h>
#include <energy.h>

namespace lstl = leuville::simple_template_library;
namespace lora = leuville::lora;

using namespace lstl;
using namespace lora;

// LoRaWAN configuration: see OTAAId in LMICWrapper.h
#include "lora-common-defs.h"

#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
USBPrinter<Serial_> console(LMIC_PRINTF_TO);
#endif

/* ---------------------------------------------------------------------------------------
 * Application classes
 * ---------------------------------------------------------------------------------------
 */

constexpr uint32_t _24h = 24 * 60 * 60;
constexpr uint32_t _7d = 7 * _24h;

/*
 * LoraWan endnode with:
 * - a timer to trigger a PING message on a regular basis
 * - a callback set on button connected to A0 pin, which triggers a BUTTON message
 * - standby mode capacity
 */
using Base = JsonEndnode;
using Button1 = ISRWrapper<DEVICE_BUTTON1_PIN>;

class EndNode : public Base, 
				private ISRTimer, 
				private Button1
{

	EnergyController<BATTPIN,BATTDIV,VOLTAGE_MIN,VOLTAGE_MAX> _energyCtrl;

	/*
	 * Jobs for LMIC event callbacks
	 */
	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
	enum JOB { BUTTON, TIMEOUT, JOIN, TXCOMPLETE, _COUNT };
	#else
	enum JOB { BUTTON, TIMEOUT, JOIN, _COUNT };
	#endif
	JobRegister<EndNode,JOB::_COUNT> _callbacks;

	int _count = 0;	// message counter

public:

	EndNode(const lmic_pinmap &pinmap): EndNode(&pinmap) {}

	EndNode(const lmic_pinmap *pinmap = Arduino_LMIC::GetPinmap_ThisBoard()): 
		Base(pinmap),
		ISRTimer(60 * DEVICE_MEASURE_DELAY, true),
		Button1(INPUT_PULLUP, LOW)
	{
		_callbacks.define(BUTTON, 		this, &EndNode::buttonJob);
		_callbacks.define(TIMEOUT, 		this, &EndNode::timeoutJob);
		_callbacks.define(JOIN, 		this, &EndNode::joinJob);
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 1
		_callbacks.define(TXCOMPLETE, 	this, &EndNode::txCompleteJob);
		#endif
	}

	/*
	 * delegates begin() to each sub-component and join
	 *
	 * ORDER is important
	 * If LMIC_USE_INTERRUPTS is enabled, LMIC must be initialized after other interruptions
	 * 
	 * Do not forget Wire.begin() if I2C devices connected
	 */
	virtual void begin(const OTAAId& id, u4_t network, bool adr = true) override {
		Wire.begin();

		_energyCtrl.setUnusedPins({A1, A2, A3, A4, A5});

		Button1::begin();
		ISRTimer::begin(); 
		ISRTimer::enable();
		Button1::enable();

		Base::begin(id, network, adr);
		startJoining();
	}

	/*
	 * Configure LoRaWan network (channels, power, adr, ...)
	 */
	virtual void initLMIC(u4_t network = 0, bool adr = true) override {
		Base::initLMIC(network, adr);
		configureNetwork(static_cast<Network>(network), adr);
	}

	/*
	 * Button ISR
	 * job done by sending a LMIC callback
	 * see completeJob()
	 */
	virtual void ISR_callback(uint8_t pin) override {
		setCallback(_callbacks[BUTTON]);
	}

	/*
	 * Timer ISR
	 * job done by sending a LMIC callback
	 * see completeJob()
	 */
	virtual uint32_t ISR_timeout() override {
		setCallback(_callbacks[TIMEOUT]);
		return 60 * DEVICE_MEASURE_DELAY;
	}

	/*
	 * Job done on join/unjoin
	 * see completeJob()
	 */
	virtual void joined(bool ok) override {
		if (ok) {
			setCallback(_callbacks[JOIN]);
		} else {
			for (auto & job : _callbacks) {
				unsetCallback(job);
			}
		}
	}

	/*
	 * Updates the system time
	 */
	#if defined(LMIC_ENABLE_DeviceTimeReq)
	virtual void updateSystemTime(uint32_t newTime) override {
		ISRTimer::setEpoch(newTime);
    	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		RTCZero & rtc = ISRTimer::getRTC();
		console.println("----------------------------------------------------------");
		console.print("heure network=",	rtc.getHours()); 
		console.print(":", rtc.getMinutes());
		console.println(":", rtc.getSeconds());
		console.println("----------------------------------------------------------");
    	#endif
	}
	#endif

	/*
	 * Updates the timer delay
	 */
	virtual void downlinkReceived(const JsonDocument & msg, const DownstreamMessage & rawMessage)  {
		if (msg["timerDelay"].is<float>()) {
			uint32_t timerDelay = 60 * msg["timerDelay"].as<float>();
			#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
			console.println("----------------------------------------------------------");
			console.println("timerDelay: ", timerDelay);
			console.println("----------------------------------------------------------");
			#endif
			ISRTimer::setTimeout(timerDelay);
		}
	}

	/*
	 * Handle LMIC callbacks
	 */
	virtual void completeJob(osjob_t* job) override {
		_callbacks[job]();
	}

	virtual void buttonJob() {
		const char* format = "CLICK %d";
		char msg[80];
		sprintf(msg, format, _energyCtrl.getBatteryPower());
		send(msg, true);
	}

	virtual void timeoutJob() {
		const char* format = "TIMEOUT %d";
		char msg[80];
		sprintf(msg, format, _count++);
		send(msg, false);
	}

	/*
	 * Build and send Uplink message
	 */
	virtual void send(String msg, bool ack = false) {
		auto batt = _energyCtrl.getBatteryPower();
		setBatteryLevel(batt);
		JsonDocument doc;
		doc["count"]        = _count++;
		doc["battery"]      = batt;
		doc["data"]         = msg;
		JsonEndnode::send(doc, ack);

		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		console.println("----------------------------------------------------------");
		console.print("send ", stringFrom(doc));
		console.println(", FIFO size: ", _messages.size());
		console.println("----------------------------------------------------------");
		#endif
	}

	virtual void joinJob() {
		LoRaWanSessionKeys keys = getSessionKeys();
		// https://www.thethingsnetwork.org/docs/lorawan/prefix-assignments.html
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		console.println("----------------------------------------------------------");
		console.println("netId: ", keys._netId, HEX);
		console.println("devAddr: ", keys._devAddr, HEX);
		console.print("nwkSKey: "); console.printHex(keys._nwkSKey, arrayCapacity(keys._nwkSKey));
		console.print("appSKey: "); console.printHex(keys._appSKey, arrayCapacity(keys._appSKey));
		console.println("----------------------------------------------------------");
		#endif
		postJoinSetup(keys._netId);
	}

	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 1

	virtual void txCompleteJob() {
		console.println("----------------------------------------------------------");
		console.println("FIFO size: ", _messages.size());
		console.println("----------------------------------------------------------");
	}
	/*
	 * LMIC callback called on TX_COMPLETE event
	 */
	virtual bool isTxCompleted(const JsonDocument & doc, const UpstreamMessage & rawMessage) {
		setCallback(_callbacks[TXCOMPLETE]);
		console.println("----------------------------------------------------------");
		console.print("isTxCompleted "); 
		console.print(" / ackRequest: ", 	rawMessage._ackRequested);
		console.print(" / ack: ", 			rawMessage.isAcknowledged());
		console.print(" / lmicError: ", 	(int32_t)rawMessage._lmicTxError);
		console.print(" / _txrxFlags: ", 	rawMessage._txrxFlags);
		console.println("----------------------------------------------------------");
		return Base::isTxCompleted(doc, rawMessage);
	}

	#endif

	/*
	 * Activates standby mode
	 */
	void standby() {
		ISRTimer::standbyMode();
	}

};

/* ---------------------------------------------------------------------------------------
 * GLOBAL OBJECTS
 * ---------------------------------------------------------------------------------------
 */

BlinkingLed statusLed(LED_BUILTIN, 500);

#if defined(LMIC_PINS)
EndNode 	endnode{LMIC_PINS}; 
#else
EndNode 	endnode{Arduino_LMIC::GetPinmap_ThisBoard()};
#endif

/* ---------------------------------------------------------------------------------------
 * ARDUINO FUNCTIONS
 * ---------------------------------------------------------------------------------------
 */
void setup()
{
	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
	console.begin(115200);
	#endif

	statusLed.begin();
	statusLed.on();

	endnode.begin(id[DEVICE_CONFIG], DEVICE_NETWORK, ADR::ON);

	delay(5000);
	statusLed.off();
}

void loop()
{
	endnode.runLoopOnce();
	if (endnode.isReadyForStandby()) {
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		statusLed.off();
		#else
		endnode.standby(); 
		#endif
	}
	else {
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		statusLed.blink();
		#endif
	}
}


