MODULE_NAME = problems
CHILD_DIRS := shared rocksample tag pushbox conttag manipulator_discrete manipulator_continuous

ifdef HAS_ROOT_MAKEFILE

problems: $(CHILD_DIRS)

include .make/template.mk

else
REDIRECT=$(MODULE_NAME)
.PHONY: $(MAKECMDGOALS) call-upwards
$(MAKECMDGOALS): call-upwards ;
call-upwards:
	@$(MAKE) --no-print-directory -C .. $(MAKECMDGOALS) REDIRECT=$(REDIRECT)
endif
