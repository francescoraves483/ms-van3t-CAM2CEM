/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "ASN1Files/CEM v1.2.1.asn"
 */

#ifndef	_CemTimestamp_H_
#define	_CemTimestamp_H_


#include "asn_application.h"

/* Including external dependencies */
#include "INTEGER.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CemTimestamp {
	CemTimestamp_utcStartOf2004	= 0,
	CemTimestamp_oneNanosecAfterUTCStartOf2004	= 1
} e_CemTimestamp;

/* CemTimestamp */
typedef INTEGER_t	 CemTimestamp_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_CemTimestamp_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_CemTimestamp;
asn_struct_free_f CemTimestamp_free;
asn_struct_print_f CemTimestamp_print;
asn_constr_check_f CemTimestamp_constraint;
ber_type_decoder_f CemTimestamp_decode_ber;
der_type_encoder_f CemTimestamp_encode_der;
xer_type_decoder_f CemTimestamp_decode_xer;
xer_type_encoder_f CemTimestamp_encode_xer;
oer_type_decoder_f CemTimestamp_decode_oer;
oer_type_encoder_f CemTimestamp_encode_oer;
per_type_decoder_f CemTimestamp_decode_uper;
per_type_encoder_f CemTimestamp_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _CemTimestamp_H_ */
#include "asn_internal.h"
