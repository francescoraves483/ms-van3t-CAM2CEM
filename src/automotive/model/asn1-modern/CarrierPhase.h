/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "CEM v1.2.2.asn"
 */

#ifndef	_CarrierPhase_H_
#define	_CarrierPhase_H_


#include "asn_application.h"

/* Including external dependencies */
#include "INTEGER.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CarrierPhase {
	CarrierPhase_unavailable	= 160000000001
} e_CarrierPhase;

/* CarrierPhase */
typedef INTEGER_t	 CarrierPhase_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_CarrierPhase_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_CarrierPhase;
asn_struct_free_f CarrierPhase_free;
asn_struct_print_f CarrierPhase_print;
asn_constr_check_f CarrierPhase_constraint;
ber_type_decoder_f CarrierPhase_decode_ber;
der_type_encoder_f CarrierPhase_encode_der;
xer_type_decoder_f CarrierPhase_decode_xer;
xer_type_encoder_f CarrierPhase_encode_xer;
oer_type_decoder_f CarrierPhase_decode_oer;
oer_type_encoder_f CarrierPhase_encode_oer;
per_type_decoder_f CarrierPhase_decode_uper;
per_type_encoder_f CarrierPhase_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _CarrierPhase_H_ */
#include "asn_internal.h"
