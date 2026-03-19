extern uint16_t uwF_INPUT;
extern uint16_t uwpreF_INPUT;
extern uint16_t uwA_INPUT;
extern int16_t  wInject;
extern uint16_t uwDetectPoint;
extern uint16_t START_BODESCAN_MARK;
extern uint16_t FINISH_BODESCAN_MARK;
extern uint32_t udwTIME_OFFSET;
extern uint16_t uwCircle;
extern uint16_t uwF_SAMP;
extern uint16_t dftn;
extern uint32_t udwTIMECNT;
extern uint32_t udwTIME_OFFSET;
extern uint16_t uwcntN;
extern float    fUnitTheta;
extern uint16_t BSCommandCnt;
extern uint16_t BTCommandCnt;
extern uint16_t BQCommandCnt;
extern uint16_t InValidCommandCnt;

void BodeInit( void );
void BodeScan( uint16_t wSample );

