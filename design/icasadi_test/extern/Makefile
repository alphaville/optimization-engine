all:
	gcc -Wall -fPIC -c -o auto_casadi_cost.o auto_casadi_cost.c;
	gcc -Wall -fPIC -c -o auto_casadi_grad.o auto_casadi_grad.c;
	gcc -Wall -fPIC -c -o icasadi.o icasadi.c;
	gcc -Wall -fPIC -c -o auto_casadi_constraints_type_penalty.o auto_casadi_constraints_type_penalty.c;
	gcc -Wall -o icasadirunner main.c auto_casadi_grad.o auto_casadi_cost.o auto_casadi_constraints_type_penalty.o icasadi.o -lm;


run: all
	./icasadirunner


clean:
	rm -rf *.o
	rm -f ./icasadirunner


.PHONY: run, clean, all, icasadirunner
