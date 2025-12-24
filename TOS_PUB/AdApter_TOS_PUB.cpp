#include"AdApter_TOS_PUB.h"

void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}



AdApter_TOSPUB::AdApter_TOSPUB()
{
}
AdApter_TOSPUB::~AdApter_TOSPUB()
{
}
auto AdApter_TOSPUB::maxeye_midware_init()
{
  return proto_info;
}


void AdApter_TOSPUB::run()
{
    int i=0;
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(LongCtrlObjInfo));
    MOS::communication::Init(json_file.c_str());
    MOS::utils::Register::get().register_version("libTOSPUB", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;//kProtocolHybrid
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
    auto pub = MOS::communication::Publisher::New(domiain_id, topic, proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();

    
    LongCtrlObjInfo TPUB_;

 
        
    std::cout << "TPUB_  size=" << sizeof(TPUB_)<< std::endl;
    print_memory(&TPUB_, sizeof(TPUB_));

    std::cout << "data_in[0]  size=" <<  data_in.size()<< std::endl;
    print_memory(&data_in[0],  data_in.size());

    std::cout << "data_in.data  size=" << data_in.size() << std::endl;
    print_memory(data_in.data(),  data_in.size());

    

// ==================== Initialize TargetsSelectedForACC Structure ====================

// 1. Initialize Left Adjacent Lane Target (AccTgtAdjLLeft)
TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.A = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.A: " << TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.A << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.Heading = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.Heading: " << TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.Heading << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.Idn = 3;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.Idn: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.Idn) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.Index = 3;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.Index: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.Index) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoad = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoad: " << TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoad << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoadConfidence = DataConfidenceLvl3_NotRelbl;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoadConfidence: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoadConfidence) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.motionHistory = MotionHistory_NotSeenMoving;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.motionHistory: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.motionHistory) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.motionPattern = MotionPattern2_Unknown;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.motionPattern: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.motionPattern) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLat = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLat: " << TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLat << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLgt = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLgt: " << TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLgt << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.Spd = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.Spd: " << TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.Spd << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.TgtLaneSts = TargetLaneStatus_TgtKeepingLane;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.TgtLaneSts: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.TgtLaneSts) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.turnIndicator = IndicatorStatus_NoIndcn;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.turnIndicator: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.turnIndicator) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.type = ObjectClass7_UkwnClass;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.type: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLLeft.type) << std::endl;

std::cout << "=== Left Adjacent Lane Target Initialization Complete ===" << std::endl << std::endl;

// 2. Initialize Right Adjacent Lane Target (AccTgtAdjLRight)
TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.A = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.A: " << TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.A << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.Heading = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.Heading: " << TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.Heading << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.Idn = 3;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.Idn: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.Idn) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.Index = 3;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.Index: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.Index) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoad = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoad: " << TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoad << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoadConfidence = DataConfidenceLvl3_NotRelbl;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoadConfidence: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoadConfidence) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.motionHistory = MotionHistory_NotSeenMoving;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.motionHistory: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.motionHistory) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.motionPattern = MotionPattern2_Unknown;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.motionPattern: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.motionPattern) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLat = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLat: " << TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLat << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLgt = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLgt: " << TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLgt << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.Spd = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.Spd: " << TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.Spd << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.TgtLaneSts = TargetLaneStatus_TgtKeepingLane;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.TgtLaneSts: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.TgtLaneSts) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.turnIndicator = IndicatorStatus_NoIndcn;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.turnIndicator: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.turnIndicator) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.type = ObjectClass7_UkwnClass;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.type: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtAdjLRight.type) << std::endl;

std::cout << "=== Right Adjacent Lane Target Initialization Complete ===" << std::endl << std::endl;

// 3. Initialize Cut-in Target (AccTgtCutIn)
TPUB_.TargetsSelectedForACC.AccTgtCutIn.A = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.A: " << TPUB_.TargetsSelectedForACC.AccTgtCutIn.A << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.Heading = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.Heading: " << TPUB_.TargetsSelectedForACC.AccTgtCutIn.Heading << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.Idn = 3;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.Idn: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtCutIn.Idn) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.Index = 3;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.Index: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtCutIn.Index) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoad = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoad: " << TPUB_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoad << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoadConfidence = DataConfidenceLvl3_NotRelbl;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoadConfidence: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoadConfidence) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.motionHistory = MotionHistory_NotSeenMoving;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.motionHistory: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtCutIn.motionHistory) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.motionPattern = MotionPattern2_Unknown;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.motionPattern: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtCutIn.motionPattern) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.PosnLat = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.PosnLat: " << TPUB_.TargetsSelectedForACC.AccTgtCutIn.PosnLat << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.PosnLgt = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.PosnLgt: " << TPUB_.TargetsSelectedForACC.AccTgtCutIn.PosnLgt << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.Spd = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.Spd: " << TPUB_.TargetsSelectedForACC.AccTgtCutIn.Spd << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.TgtLaneSts = TargetLaneStatus_TgtKeepingLane;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.TgtLaneSts: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtCutIn.TgtLaneSts) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.turnIndicator = IndicatorStatus_NoIndcn;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.turnIndicator: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtCutIn.turnIndicator) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtCutIn.type = ObjectClass7_UkwnClass;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtCutIn.type: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtCutIn.type) << std::endl;

std::cout << "=== Cut-in Target Initialization Complete ===" << std::endl << std::endl;

// 4. Initialize First Closest Lane Target (AccTgtFrstClstLane) - Set test values
TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.A = 1.1f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.A: " << TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.A << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.Heading = 3.5f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.Heading: " << TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.Heading << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.Idn = 1;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.Idn: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.Idn) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.Index = 1;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.Index: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.Index) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoad = 2.5f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoad: " << TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoad << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoadConfidence = DataConfidenceLvl3_HighestRelbl;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoadConfidence: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoadConfidence) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.motionHistory = MotionHistory_SeenReceding;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.motionHistory: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.motionHistory) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.motionPattern = MotionPattern2_Receding;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.motionPattern: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.motionPattern) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLat = 3.0f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLat: " << TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLat << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLgt = 50.0f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLgt: " << TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLgt << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.Spd = 10.0f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.Spd: " << TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.Spd << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.TgtLaneSts = TargetLaneStatus_TgtKeepingLane;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.TgtLaneSts: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.TgtLaneSts) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.turnIndicator = IndicatorStatus_NoIndcn;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.turnIndicator: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.turnIndicator) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.type = ObjectClass7_Car;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.type: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtFrstClstLane.type) << std::endl;

std::cout << "=== First Closest Lane Target Initialization Complete ===" << std::endl << std::endl;

// 5. Initialize Second Closest Lane Target (AccTgtSecClstLane)
TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.A = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.A: " << TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.A << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.Heading = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.Heading: " << TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.Heading << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.Idn = 3;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.Idn: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.Idn) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.Index = 3;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.Index: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.Index) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoad = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoad: " << TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoad << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoadConfidence = DataConfidenceLvl3_NotRelbl;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoadConfidence: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoadConfidence) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.motionHistory = MotionHistory_NotSeenMoving;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.motionHistory: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.motionHistory) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.motionPattern = MotionPattern2_Unknown;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.motionPattern: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.motionPattern) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLat = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLat: " << TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLat << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLgt = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLgt: " << TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLgt << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.Spd = 1.2f;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.Spd: " << TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.Spd << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.TgtLaneSts = TargetLaneStatus_TgtKeepingLane;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.TgtLaneSts: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.TgtLaneSts) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.turnIndicator = IndicatorStatus_NoIndcn;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.turnIndicator: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.turnIndicator) << std::endl;

TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.type = ObjectClass7_UkwnClass;
std::cout << "TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.type: " << static_cast<int>(TPUB_.TargetsSelectedForACC.AccTgtSecClstLane.type) << std::endl;

std::cout << "=== Second Closest Lane Target Initialization Complete ===" << std::endl << std::endl;

// ==================== Initialize AdditionalTarSelnSignals Structure ====================

TPUB_.AdditionalTarSelnSignals.Signal1 = 2.2f;
std::cout << "TPUB_.AdditionalTarSelnSignals.Signal1: " << TPUB_.AdditionalTarSelnSignals.Signal1 << std::endl;

TPUB_.AdditionalTarSelnSignals.Signal2 = 1.2f;
std::cout << "TPUB_.AdditionalTarSelnSignals.Signal2: " << TPUB_.AdditionalTarSelnSignals.Signal2 << std::endl;

TPUB_.AdditionalTarSelnSignals.Signal3 = 1.2f;
std::cout << "TPUB_.AdditionalTarSelnSignals.Signal3: " << TPUB_.AdditionalTarSelnSignals.Signal3 << std::endl;

TPUB_.AdditionalTarSelnSignals.Signal4 = 1.2f;
std::cout << "TPUB_.AdditionalTarSelnSignals.Signal4: " << TPUB_.AdditionalTarSelnSignals.Signal4 << std::endl;

TPUB_.AdditionalTarSelnSignals.Signal5 = 1.2f;
std::cout << "TPUB_.AdditionalTarSelnSignals.Signal5: " << TPUB_.AdditionalTarSelnSignals.Signal5 << std::endl;

TPUB_.AdditionalTarSelnSignals.Signal6 = 1.2f;
std::cout << "TPUB_.AdditionalTarSelnSignals.Signal6: " << TPUB_.AdditionalTarSelnSignals.Signal6 << std::endl;

TPUB_.AdditionalTarSelnSignals.Signal7 = 1.2f;
std::cout << "TPUB_.AdditionalTarSelnSignals.Signal7: " << TPUB_.AdditionalTarSelnSignals.Signal7 << std::endl;

TPUB_.AdditionalTarSelnSignals.Signal8 = 1.2f;
std::cout << "TPUB_.AdditionalTarSelnSignals.Signal8: " << TPUB_.AdditionalTarSelnSignals.Signal8 << std::endl;

//TPUB_.Additional


    while (true) 
    {
         
        data_in.resize(sizeof(TPUB_));
        std::memcpy(&data_in[0], &TPUB_, sizeof(TPUB_));

        auto data_ref = std::make_shared<MOS::message::DataRef>(const_cast<char*>(data_in.data()), data_in.size());

        mos_msg->SetDataRef(data_ref);
        auto now_time = MOS::TimeUtils::NowNsec();
        mos_msg->SetGenTimestamp(now_time);
        pub->Pub(mos_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms));//40ms

        //std::cout << "data_ref  size=" <<  data_ref->GetDataSizeVec().back() << std::endl;//204,OK
        //print_memory(data_ref->GetDataVec().back(),   data_ref->GetDataSizeVec().back()  );//OK


    }




   
}


int main()
{
#ifdef _WIN32
    char* path = nullptr;
    _get_pgmptr(&path);
    std::stringstream ss;
    ss << path;
    std::string fullPath(path);
    std::cout << "Running on Windows" << std::endl;
#endif
#ifdef __linux__
    std::string fullPath = std::filesystem::read_symlink("/proc/self/exe");
    std::cout << "Running PPF_PUB on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_TOSPUB objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
