import pandas as pd
a=[]
b=[]
df = pd.read_csv('responses.csv')
df_val = pd.read_csv('values.csv')


for i in range(17,53):
    data = pd.DataFrame(df.iloc[1:,i]).dropna()
    h=data.columns.tolist()[0]
    l=list(data[h])
    for k in l:
       a.append(k)
    val=[df_val.iloc[i-17,0]]
    val=len(l)*val
    for v in val:
     b.append(v)
p=pd.DataFrame()
p['input']=a
p['output']=b
p.to_csv('scrapedfile2.csv', index=False)