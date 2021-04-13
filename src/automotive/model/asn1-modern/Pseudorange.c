/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "ASN1Files/CEM v1.2.1.asn"
 */

#include "Pseudorange.h"

int
Pseudorange_constraint(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const INTEGER_t *st = (const INTEGER_t *)sptr;
	unsigned long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	if(asn_INTEGER2ulong(st, &value)) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value too large (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	if((value >= 1800000000 && value <= 5900000001)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

/*
 * This type is implemented using INTEGER,
 * so here we adjust the DEF accordingly.
 */
static asn_oer_constraints_t asn_OER_type_Pseudorange_constr_1 CC_NOTUSED = {
	{ 4, 1 }	/* (1800000000..2900000001) */,
	-1};
asn_per_constraints_t asn_PER_type_Pseudorange_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 31, -1,  1800000000,  5900000001 }	/* (1800000000..2900000001) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const ber_tlv_tag_t asn_DEF_Pseudorange_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (2 << 2))
};
asn_TYPE_descriptor_t asn_DEF_Pseudorange = {
	"Pseudorange",
	"Pseudorange",
	&asn_OP_INTEGER,
	asn_DEF_Pseudorange_tags_1,
	sizeof(asn_DEF_Pseudorange_tags_1)
		/sizeof(asn_DEF_Pseudorange_tags_1[0]), /* 1 */
	asn_DEF_Pseudorange_tags_1,	/* Same as above */
	sizeof(asn_DEF_Pseudorange_tags_1)
		/sizeof(asn_DEF_Pseudorange_tags_1[0]), /* 1 */
	{ &asn_OER_type_Pseudorange_constr_1, &asn_PER_type_Pseudorange_constr_1, Pseudorange_constraint },
	0, 0,	/* Defined elsewhere */
	0	/* No specifics */
};

