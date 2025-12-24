#include"AdApter_SOC_ActiveSafety_UDP_T_T.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<SOC_ActiveSafety_UDP_T_TModule::SOC_ActiveSafety_UDP_T_T> AdApter_SOC_ActiveSafety_UDP_T_T::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<SOC_ActiveSafety_UDP_T_TModule::SOC_ActiveSafety_UDP_T_T> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<SOC_ActiveSafety_UDP_T_TModule::SOC_ActiveSafety_UDP_T_T> writer(publisher, topic);
    return writer;
}
void AdApter_SOC_ActiveSafety_UDP_T_T::update_objInfo(std::string data_str)
{
    SOC_ActiveSafety_UDP_T_T SOC_ActiveSafety_UDP_T_T_;
    std::memcpy(&SOC_ActiveSafety_UDP_T_T_, data_str.data(), sizeof(SOC_ActiveSafety_UDP_T_T));
    topic_out.frameID() = SOC_ActiveSafety_UDP_T_T_.frameID;
    topic_out.SOC_ActiveSafety_SOC().SOC_FCW_Flag().FCW_State() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_FCW_Flag.FCW_State;
    topic_out.SOC_ActiveSafety_SOC().SOC_FCW_Flag().FCW_WarningState() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_FCW_Flag.FCW_WarningState;
    topic_out.SOC_ActiveSafety_SOC().SOC_FCW_Flag().FCW_LatentWarning() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_FCW_Flag.FCW_LatentWarning;
    topic_out.SOC_ActiveSafety_SOC().SOC_FCW_Flag().FCW_AcuteWarningState() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_FCW_Flag.FCW_AcuteWarningState;
    topic_out.SOC_ActiveSafety_SOC().SOC_FCW_Flag().FCW_RLC() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_FCW_Flag.FCW_RLC;
    topic_out.SOC_ActiveSafety_SOC().SOC_FCW_Flag().FCW_SWC_MajorVer() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_FCW_Flag.FCW_SWC_MajorVer;
    topic_out.SOC_ActiveSafety_SOC().SOC_FCW_Flag().FCW_SWC_MinorVer() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_FCW_Flag.FCW_SWC_MinorVer;
    topic_out.SOC_ActiveSafety_SOC().SOC_FCW_Flag().FCW_Param_MajorVer() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_FCW_Flag.FCW_Param_MajorVer;
    topic_out.SOC_ActiveSafety_SOC().SOC_FCW_Flag().FCW_Param_MinorVer() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_FCW_Flag.FCW_Param_MinorVer;
    topic_out.SOC_ActiveSafety_SOC().SOC_FCW_Flag().FCW_Cal_MajorVer() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_FCW_Flag.FCW_Cal_MajorVer;
    topic_out.SOC_ActiveSafety_SOC().SOC_FCW_Flag().FCW_Cal_MinorVer() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_FCW_Flag.FCW_Cal_MinorVer;
    topic_out.SOC_ActiveSafety_SOC().SOC_FCW_Flag().reserved() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_FCW_Flag.reserved;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().AEB_FunConfig() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.AEB_FunConfig;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().AEB_State() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.AEB_State;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().AEB_PrefillReq() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.AEB_PrefillReq;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().AEB_Level() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.AEB_Level;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().AEB_Type() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.AEB_Type;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().AEB_RLC() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.AEB_RLC;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().AEB_MajorVer() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.AEB_MajorVer;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().AEB_MinorVer() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.AEB_MinorVer;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().AEB_Param_MajorVer() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.AEB_Param_MajorVer;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().AEB_Param_MinorVer() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.AEB_Param_MinorVer;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().AEB_Cal_MajorVer() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.AEB_Cal_MajorVer;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().AEB_Cal_MinorVer() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.AEB_Cal_MinorVer;
    topic_out.SOC_ActiveSafety_SOC().SOC_AEB_Flag().reserved() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_AEB_Flag.reserved;
    topic_out.SOC_ActiveSafety_SOC().SOC_DRT().DRT_TTC() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_DRT.DRT_TTC;
    topic_out.SOC_ActiveSafety_SOC().SOC_DRT().DRT_ID() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_DRT.DRT_ID;
    topic_out.SOC_ActiveSafety_SOC().SOC_DRT().DRT_ObjectClass() = SOC_ActiveSafety_UDP_T_T_.SOC_ActiveSafety_SOC.SOC_DRT.DRT_ObjectClass;
    for (int i = 0; i < 2; i++)
    {
    }
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_ReactionTime() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_ReactionTime;
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_TTCThres() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_TTCThres;
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_TTC() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_TTC;
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_SCPComponent() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_SCPComponent;
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_GOComponent() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_GOComponent;
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_RLC() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_RLC;
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_State() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_State;
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_StateLastMoment() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_StateLastMoment;
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_AlrtPr() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_AlrtPr;
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_ChimeSuppress() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_ChimeSuppress;
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_BrakeSuppress() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_BrakeSuppress;
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_StatComponent() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_StatComponent;
    topic_out.FCW_Manager_SOC().FCW_DEV().FCW_MovComponent() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DEV.FCW_MovComponent;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTRange() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTRange;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTRangeRate() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTRangeRate;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTXolc() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTXolc;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTXohp() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTXohp;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTLatVel() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTLatVel;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTAngle() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTAngle;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTRangeAccel() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTRangeAccel;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTTTC() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTTTC;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTTTCThres() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTTTCThres;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTMatchConf() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTMatchConf;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTID() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTID;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTClass() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTClass;
    topic_out.FCW_Manager_SOC().FCW_DSRT().FCW_DSRTGOEConfirm() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DSRT.FCW_DSRTGOEConfirm;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTRange() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTRange;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTRangeRate() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTRangeRate;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTXolc() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTXolc;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTXohp() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTXohp;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTLatVel() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTLatVel;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTAngle() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTAngle;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTRangeAccel() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTRangeAccel;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTTTC() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTTTC;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTTTCThres() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTTTCThres;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTMatchConf() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTMatchConf;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTID() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTID;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTClass() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTClass;
    topic_out.FCW_Manager_SOC().FCW_DMRT().FCW_DMRTGOEConfirm() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_DMRT.FCW_DMRTGOEConfirm;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehHeading() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehHeading;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehLatPosCoG() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehLatPosCoG;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehLongPosCoG() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehLongPosCoG;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehLatVel() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehLatVel;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehLongVel() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehLongVel;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehLatAcc() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehLatAcc;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehLongAcc() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehLongAcc;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehPathDisNear() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehPathDisNear;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehXolcRef1() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehXolcRef1;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehXolcRef2() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehXolcRef2;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehTTC() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehTTC;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehTTCThres() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehTTCThres;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().FCW_TAPVehId() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.FCW_TAPVehId;
    topic_out.FCW_Manager_SOC().FCW_TAPVeh().ByteAlignedFill() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVeh.ByteAlignedFill;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRUHeading() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRUHeading;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRULatPosCoG() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRULatPosCoG;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRULongPosCoG() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRULongPosCoG;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRULatVel() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRULatVel;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRULongVel() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRULongVel;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRULatAcc() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRULatAcc;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRULongAcc() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRULongAcc;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRUPathDisNear() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRUPathDisNear;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRUXolcRef1() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRUXolcRef1;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRUXolcRef2() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRUXolcRef2;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRUTTC() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRUTTC;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRUTTCThres() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRUTTCThres;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().FCW_TAPVRUId() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.FCW_TAPVRUId;
    topic_out.FCW_Manager_SOC().FCW_TAPVRU().ByteAlignedFill() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TAPVRU.ByteAlignedFill;
    topic_out.FCW_Manager_SOC().FCW_GO().FCW_GOLongPos() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_GO.FCW_GOLongPos;
    topic_out.FCW_Manager_SOC().FCW_GO().FCW_GOLatPos() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_GO.FCW_GOLatPos;
    topic_out.FCW_Manager_SOC().FCW_GO().FCW_GOXolcRef() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_GO.FCW_GOXolcRef;
    topic_out.FCW_Manager_SOC().FCW_GO().FCW_GOTTC() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_GO.FCW_GOTTC;
    topic_out.FCW_Manager_SOC().FCW_GO().FCW_GOTTCThres() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_GO.FCW_GOTTCThres;
    topic_out.FCW_Manager_SOC().FCW_GO().FCW_GOId() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_GO.FCW_GOId;
    topic_out.FCW_Manager_SOC().FCW_GO().FCW_GOClass() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_GO.FCW_GOClass;
    topic_out.FCW_Manager_SOC().FCW_GO().ByteAlignedFill() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_GO.ByteAlignedFill;
    topic_out.FCW_Manager_SOC().FCW_Version().FCW_MajorVer() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_Version.FCW_MajorVer;
    topic_out.FCW_Manager_SOC().FCW_Version().FCW_MinorVer() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_Version.FCW_MinorVer;
    topic_out.FCW_Manager_SOC().FCW_Version().ByteAlignedFill() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_Version.ByteAlignedFill;
    topic_out.FCW_Manager_SOC().FCW_LatentWarning() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_LatentWarning;
    topic_out.FCW_Manager_SOC().FCW_AcuteTargetId() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_AcuteTargetId;
    topic_out.FCW_Manager_SOC().FCW_LatentWarningID() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_LatentWarningID;
    topic_out.FCW_Manager_SOC().FCW_WarningState() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_WarningState;
    topic_out.FCW_Manager_SOC().FCW_TargetId() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_TargetId;
    topic_out.FCW_Manager_SOC().FCW_AcuteWarningState() = SOC_ActiveSafety_UDP_T_T_.FCW_Manager_SOC.FCW_AcuteWarningState;
    for (int i = 0; i < 3; i++)
    {
    }
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTTTC() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTTTC;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTRange() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTRange;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTRangeRate() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTRangeRate;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTRangeAccel() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTRangeAccel;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTLatPosn() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTLatPosn;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTLatVel() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTLatVel;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTXolc() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTXolc;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_VRULatEst() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_VRULatEst;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTVisLatAcc() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTVisLatAcc;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTWidth() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTWidth;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_PrefillRangeThres() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_PrefillRangeThres;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_LowBrkRangeThres() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_LowBrkRangeThres;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_HiBrkRangeThres() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_HiBrkRangeThres;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_IBARangeThres() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_IBARangeThres;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_PrefillTTCThres() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_PrefillTTCThres;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_LowBrkTTCThres() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_LowBrkTTCThres;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_HiBrkTTCThres() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_HiBrkTTCThres;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_IBATTCThres() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_IBATTCThres;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_HostVehSpd_mps() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_HostVehSpd_mps;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_HostLongAccel_mpss() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_HostLongAccel_mpss;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_BrkPdlPosRate() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_BrkPdlPosRate;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_BrkPdlPrsRate() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_BrkPdlPrsRate;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_GasPedalPos() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_GasPedalPos;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_SingleRes01() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_SingleRes01;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_SingleRes02() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_SingleRes02;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_SingleRes03() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_SingleRes03;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DebugInfo1() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DebugInfo1;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DebugInfo2() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DebugInfo2;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_VehEachTrkCntr() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_VehEachTrkCntr;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_HiBrkCntr() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_HiBrkCntr;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_LowBrkCntr() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_LowBrkCntr;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_PrefillCntr() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_PrefillCntr;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_FLBACntr() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_FLBACntr;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_MinStopCntr() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_MinStopCntr;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_IBACntr() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_IBACntr;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DIDConfig1R() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DIDConfig1R;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_Uint32Res01() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_Uint32Res01;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_Uint32Res02() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_Uint32Res02;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_Uint32Res03() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_Uint32Res03;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_PrefillFlg() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_PrefillFlg;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_LowBrkFlg() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_LowBrkFlg;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_HiBrkFlg() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_HiBrkFlg;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_FullBrkFlg() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_FullBrkFlg;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_IBAFlg() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_IBAFlg;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_FLBAFlg() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_FLBAFlg;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_AEBRlsCmd() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_AEBRlsCmd;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_AEBBrkLvlReq() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_AEBBrkLvlReq;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_AEBSourceType() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_AEBSourceType;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_JAAEBActiveFlag() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_JAAEBActiveFlag;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTID() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTID;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTMovement() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTMovement;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTMoveable() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTMoveable;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTMatchConf() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTMatchConf;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTFusSource() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTFusSource;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTObjectClass() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTObjectClass;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_UseVRU() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_UseVRU;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTGOEConfirm() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTGOEConfirm;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DRTBeyondGuardrail() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DRTBeyondGuardrail;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_DrvrEng() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_DrvrEng;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_GasPedalOvrd() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_GasPedalOvrd;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_SteeringOvrd() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_SteeringOvrd;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_GasPedalRateCntr() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_GasPedalRateCntr;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_BrkPosAct() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_BrkPosAct;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_BrkPressureAct() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_BrkPressureAct;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_VehObjPrcCntr() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_VehObjPrcCntr;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_VehToiDscrmntnCntr() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_VehToiDscrmntnCntr;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_VehBrkDscrmntnCntr() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_VehBrkDscrmntnCntr;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_RLC() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_RLC;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_Uint8Res01() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_Uint8Res01;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_Uint8Res02() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_Uint8Res02;
    topic_out.LgSafe_SOC().LgSafe_DEV().LgSf_Uint8Res03() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.LgSf_Uint8Res03;
    topic_out.LgSafe_SOC().LgSafe_DEV().ByteAlignedFill() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_DEV.ByteAlignedFill;
    topic_out.LgSafe_SOC().LgSafe_Version().LgSf_MajorVer() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_Version.LgSf_MajorVer;
    topic_out.LgSafe_SOC().LgSafe_Version().LgSf_MinorVer() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_Version.LgSf_MinorVer;
    topic_out.LgSafe_SOC().LgSafe_Version().ByteAlignedFill() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSafe_Version.ByteAlignedFill;
    topic_out.LgSafe_SOC().LgSf_AEBDecelReq() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSf_AEBDecelReq;
    topic_out.LgSafe_SOC().LgSf_AEBType() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSf_AEBType;
    topic_out.LgSafe_SOC().LgSf_PrefillReq() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSf_PrefillReq;
    topic_out.LgSafe_SOC().LgSf_AEBDecelReqFlag() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSf_AEBDecelReqFlag;
    topic_out.LgSafe_SOC().LgSf_AEBBrkReqFlag() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSf_AEBBrkReqFlag;
    topic_out.LgSafe_SOC().LgSf_AEBAlrtReq() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSf_AEBAlrtReq;
    topic_out.LgSafe_SOC().LgSf_AEBVRUAlrtReq() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSf_AEBVRUAlrtReq;
    topic_out.LgSafe_SOC().LgSf_FLBAReq() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSf_FLBAReq;
    topic_out.LgSafe_SOC().LgSf_MinStopReq() = SOC_ActiveSafety_UDP_T_T_.LgSafe_SOC.LgSf_MinStopReq;
    data_in.resize(sizeof(SOC_ActiveSafety_UDP_T_T));
    std::memcpy(&data_in[0], &SOC_ActiveSafety_UDP_T_T_, sizeof(SOC_ActiveSafety_UDP_T_T));
}
void AdApter_SOC_ActiveSafety_UDP_T_T::maxeye_data_init()
{
}
AdApter_SOC_ActiveSafety_UDP_T_T::AdApter_SOC_ActiveSafety_UDP_T_T()
{
}
AdApter_SOC_ActiveSafety_UDP_T_T::~AdApter_SOC_ActiveSafety_UDP_T_T()
{
}
auto AdApter_SOC_ActiveSafety_UDP_T_T::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(SOC_ActiveSafety_UDP_T_T));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libSOC_ActiveSafety_UDP_T_T", "1.1.0");
MOS::communication::ProtocolInfo proto_info;
if (type) {
    proto_info.protocol_type = MOS::communication::kProtocolNet;
    if (!MOS::communication::IsDynamic()) {
        proto_info.net_info.remote_addr_vec.emplace_back(ip, port);
    }
}
else {
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
}
return proto_info;
}
int config_async_sub(std::string json_file) {
    AdApter_SOC_ActiveSafety_UDP_T_T AdApter_SOC_ActiveSafety_UDP_T_T_;
    AdApter_SOC_ActiveSafety_UDP_T_T_.json_file = json_file;
   dds::pub::DataWriter<SOC_ActiveSafety_UDP_T_TModule::SOC_ActiveSafety_UDP_T_T> writer = AdApter_SOC_ActiveSafety_UDP_T_T_.topic_writer_init(AdApter_SOC_ActiveSafety_UDP_T_T_.topicName);
    int domain_id = AdApter_SOC_ActiveSafety_UDP_T_T_.domiain_id;
    std::string topic = AdApter_SOC_ActiveSafety_UDP_T_T_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libSOC_ActiveSafety_UDP_T_T", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_SOC_ActiveSafety_UDP_T_T_.domiain_id,AdApter_SOC_ActiveSafety_UDP_T_T_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
    auto data_vec = tmp->GetDataRef()->GetDataVec();
    auto data_size_vec = tmp->GetDataRef()->GetDataSizeVec();
    auto size = data_size_vec.size();
    for (int i = 0; i < size; i++) {
    auto vec_size = data_size_vec[i];
    uint8_t* vec_data = static_cast<uint8_t*>(data_vec[i]);
    data_in = std::string(reinterpret_cast<const char*>(vec_data), vec_size);
    }
  });
    while (!stop.load()) {
        AdApter_SOC_ActiveSafety_UDP_T_T_.update_objInfo(data_in);
        writer.write(AdApter_SOC_ActiveSafety_UDP_T_T_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_SOC_ActiveSafety_UDP_T_T_.cycle_ms));
    }
    return 0;
    }
void AdApter_SOC_ActiveSafety_UDP_T_T::run()
{
    config_async_sub(json_file);
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
    std::cout << "Running on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_SOC_ActiveSafety_UDP_T_T objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
