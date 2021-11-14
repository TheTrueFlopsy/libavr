
service_name := inm_router
#start_script := start_inm_router.sh
executable := inm_router.py
library_code := inm.py
init_script := inm_router.sh
config_file := router_args
defaults_file := inm_router.defaults

init_dir := /etc/init.d/
defaults_dir := /etc/default/
install_dir := /usr/local/
bin_dir := ${install_dir}bin/
lib_dir := ${install_dir}lib/inm/
config_dir := ${install_dir}etc/inm/


.PHONY : clean install

clean :
	${RM} ${defaults_file}

${defaults_file} :
	echo 'PYTHONPATH="$$PYTHONPATH:${lib_dir}"' > ${defaults_file}

install : ${config_file} ${defaults_file}
	install -d ${bin_dir} ${lib_dir} ${config_dir}
	install -m 644 ${library_code} ${lib_dir}
	install -m 644 ${config_file} ${config_dir}
#	install ${start_script} ${bin_dir}
	install ${executable} ${bin_dir}
	install -m 644 -T ${defaults_file} ${defaults_dir}${service_name}
	install -T ${init_script} ${init_dir}${service_name}
	update-rc.d ${service_name} defaults
