/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CEM-PDU-Descriptions"
 * 	found in "CEM v1.2.2.asn"
 */

#include "CoopEnhancement.h"

static asn_oer_constraints_t asn_OER_type_CoopEnhancement_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
asn_per_constraints_t asn_PER_type_CoopEnhancement_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  1,  1,  0,  1 }	/* (0..1,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
asn_TYPE_member_t asn_MBR_CoopEnhancement_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct CoopEnhancement, choice.fps),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FullPrecisionInterframe,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"fps"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CoopEnhancement, choice.dmf),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DifferentialMicroframe,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"dmf"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_CoopEnhancement_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* fps */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* dmf */
};
asn_CHOICE_specifics_t asn_SPC_CoopEnhancement_specs_1 = {
	sizeof(struct CoopEnhancement),
	offsetof(struct CoopEnhancement, _asn_ctx),
	offsetof(struct CoopEnhancement, present),
	sizeof(((struct CoopEnhancement *)0)->present),
	asn_MAP_CoopEnhancement_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0,
	2	/* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_CoopEnhancement = {
	"CoopEnhancement",
	"CoopEnhancement",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ &asn_OER_type_CoopEnhancement_constr_1, &asn_PER_type_CoopEnhancement_constr_1, CHOICE_constraint },
	asn_MBR_CoopEnhancement_1,
	2,	/* Elements count */
	&asn_SPC_CoopEnhancement_specs_1	/* Additional specs */
};

