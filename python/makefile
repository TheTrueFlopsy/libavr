
# --- Service Config ---
service_name := inm_router
executables := inm_router.py inm_memmon.py
pkg_name := inm
mod_names := __init__ inm helper
library_code := $(addprefix $(pkg_name)/, $(addsuffix .py, $(mod_names)))
init_script := inm_router.sh
config_file := router_args
defaults_file := inm_router.defaults

init_dir := /etc/init.d/
defaults_dir := /etc/default/
install_dir := /usr/local/
bin_dir := $(install_dir)bin/
lib_dir := $(install_dir)lib/inm/
pkg_dir := $(lib_dir)$(pkg_name)/
config_dir := $(install_dir)etc/inm/

# --- API Docs Config ---
natural := naturaldocs
natural_flags := --highlight all --style Default MyStyle

doc_dir := docs/natural
doc_conf_dir := docs/natural-config
docdirs := $(doc_dir) $(doc_conf_dir)

# --- Targets ---
.PHONY : clean doc doc-rebuild install

# ISSUE: Delete __pycache__ directories when cleaning?
clean :
	$(RM) $(defaults_file)

doc-rebuild : natural_flags += --rebuild
doc-rebuild : doc

doc : $(library_code) | $(docdirs)
	$(natural) -i . -o HTML $(doc_dir) -p $(doc_conf_dir) $(natural_flags)

# TODO: Precompile the installed Python source? Build an archive?
install : $(config_file) $(defaults_file)
	install -d $(bin_dir) $(pkg_dir) $(config_dir)
	install -m 644 $(library_code) $(pkg_dir)
	install -m 644 $(config_file) $(config_dir)
	install $(executables) $(bin_dir)
	install -m 644 -T $(defaults_file) $(defaults_dir)$(service_name)
	install -T $(init_script) $(init_dir)$(service_name)
	update-rc.d $(service_name) defaults

$(docdirs) :
	mkdir -p $@

$(defaults_file) :
	echo 'PYTHONPATH="$$PYTHONPATH:$(lib_dir)"' > $(defaults_file)
