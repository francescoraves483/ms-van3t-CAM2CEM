/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "CEM v1.2.2.asn"
 */

#ifndef	_SignalStrength_H_
#define	_SignalStrength_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SignalStrength {
	SignalStrength_unavailable	= 201
} e_SignalStrength;

/* SignalStrength */
typedef long	 SignalStrength_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_SignalStrength_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_SignalStrength;
asn_struct_free_f SignalStrength_free;
asn_struct_print_f SignalStrength_print;
asn_constr_check_f SignalStrength_constraint;
ber_type_decoder_f SignalStrength_decode_ber;
der_type_encoder_f SignalStrength_encode_der;
xer_type_decoder_f SignalStrength_decode_xer;
xer_type_encoder_f SignalStrength_encode_xer;
oer_type_decoder_f SignalStrength_decode_oer;
oer_type_encoder_f SignalStrength_encode_oer;
per_type_decoder_f SignalStrength_decode_uper;
per_type_encoder_f SignalStrength_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _SignalStrength_H_ */
#include "asn_internal.h"
