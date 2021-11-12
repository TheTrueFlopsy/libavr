
#include "std_tlv.h"

ttlv_state ttlv_xmit_response(uint8_t type, uint8_t length, const uint8_t *data_p) {
	return ttlv_xmit(ttlv_recv_inm_header.h.srcadr, type, length, data_p);
}

ttlv_state ttlv_xmit_result(ttlv_result res) {
	return ttlv_xmit(
		ttlv_recv_inm_header.h.srcadr, TTLV_MSG_T_RESULT, TTLV_MSG_L_RESULT,
		TTLV_DATA_PTR(&res));
}

ttlv_state ttlv_xmit_inm_result(ttlv_result res) {
	ttlv_msg_inm_result payload = {
		.result_code = res,
		.request_id = ttlv_recv_inm_header.h.msg_id
	};
	
	return ttlv_xmit(
		ttlv_recv_inm_header.h.srcadr, TTLV_MSG_T_INM_RESULT, TTLV_MSG_L_INM_RESULT,
		TTLV_DATA_PTR(&payload));
}
