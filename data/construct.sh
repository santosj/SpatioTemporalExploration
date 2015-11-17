for i in $(seq 1 $1);do 
	cat joao.data |sed -n $(($i*288)),$((($i+1)*288-1))p|sed s/$/+0\)%4/|sed s/^/\(/|bc >$i.txt;
done
for i in $(seq $(($1+1)) $(($1+$2)));do
	cat joao.data |sed -n $(($i*288)),$((($i+1)*288-1))p|sed s/$/+0\)%2/|sed s/^/\(/|bc >$i.txt;
done
for i in $(ls ?.txt);do
	mv $i 0$i
done
paste ??.txt >all.txt
rm ??.txt
