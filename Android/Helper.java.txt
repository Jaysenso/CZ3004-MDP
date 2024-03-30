package com.example.mdp;

import java.util.HashMap;
import java.util.Map;

public class Helper {
  protected static final String ROBOT = "ROBOT";
  protected static final String TARGET = "TARGET";
  protected static final String STATUS = "STATUS";
  protected static final String PLOT = "PLOT";
  protected static final String COMMAND = "COMMAND";

  protected static Map<String, Integer> resources = new HashMap<String, Integer>() {
    {
      put("o1n", R.drawable.obs1n);
      put("o1e", R.drawable.obs1e);
      put("o1s", R.drawable.obs1s);
      put("o1w", R.drawable.obs1w);

      put("o2n", R.drawable.obs2n);
      put("o2e", R.drawable.obs2e);
      put("o2s", R.drawable.obs2s);
      put("o2w", R.drawable.obs2w);

      put("o3n", R.drawable.obs3n);
      put("o3e", R.drawable.obs3e);
      put("o3s", R.drawable.obs3s);
      put("o3w", R.drawable.obs3w);

      put("o4n", R.drawable.obs4n);
      put("o4e", R.drawable.obs4e);
      put("o4s", R.drawable.obs4s);
      put("o4w", R.drawable.obs4w);

      put("o5n", R.drawable.obs5n);
      put("o5e", R.drawable.obs5e);
      put("o5s", R.drawable.obs5s);
      put("o5w", R.drawable.obs5w);

      put("o6n", R.drawable.obs6n);
      put("o6e", R.drawable.obs6e);
      put("o6s", R.drawable.obs6s);
      put("o6w", R.drawable.obs6w);

      put("o7n", R.drawable.obs7n);
      put("o7e", R.drawable.obs7e);
      put("o7s", R.drawable.obs7s);
      put("o7w", R.drawable.obs7w);

      put("o8n", R.drawable.obs8n);
      put("o8e", R.drawable.obs8e);
      put("o8s", R.drawable.obs8s);
      put("o8w", R.drawable.obs8w);

      put("11", R.drawable.num1);
      put("11n", R.drawable.num1n);
      put("11e", R.drawable.num1e);
      put("11s", R.drawable.num1s);
      put("11w", R.drawable.num1w);

      put("12", R.drawable.num2);
      put("12n", R.drawable.num2n);
      put("12e", R.drawable.num2e);
      put("12s", R.drawable.num2s);
      put("12w", R.drawable.num2w);

      put("13", R.drawable.num3);
      put("13n", R.drawable.num3n);
      put("13e", R.drawable.num3e);
      put("13s", R.drawable.num3s);
      put("13w", R.drawable.num3w);

      put("14", R.drawable.num4);
      put("14n", R.drawable.num4n);
      put("14e", R.drawable.num4e);
      put("14s", R.drawable.num4s);
      put("14w", R.drawable.num4w);

      put("15", R.drawable.num5);
      put("15n", R.drawable.num5n);
      put("15e", R.drawable.num5e);
      put("15s", R.drawable.num5s);
      put("15w", R.drawable.num5w);

      put("16", R.drawable.num6);
      put("16n", R.drawable.num6n);
      put("16e", R.drawable.num6e);
      put("16s", R.drawable.num6s);
      put("16w", R.drawable.num6w);

      put("17", R.drawable.num7);
      put("17n", R.drawable.num7n);
      put("17e", R.drawable.num7e);
      put("17s", R.drawable.num7s);
      put("17w", R.drawable.num7w);

      put("18", R.drawable.num8);
      put("18n", R.drawable.num8n);
      put("18e", R.drawable.num8e);
      put("18s", R.drawable.num8s);
      put("18w", R.drawable.num8w);

      put("19", R.drawable.num9);
      put("19n", R.drawable.num9n);
      put("19e", R.drawable.num9e);
      put("19s", R.drawable.num9s);
      put("19w", R.drawable.num9w);

      put("20", R.drawable.alphA);
      put("20n", R.drawable.alphAn);
      put("20e", R.drawable.alphAe);
      put("20s", R.drawable.alphAs);
      put("20w", R.drawable.alphAw);

      put("21", R.drawable.alphB);
      put("21n", R.drawable.alphBn);
      put("21e", R.drawable.alphBe);
      put("21s", R.drawable.alphBs);
      put("21w", R.drawable.alphBw);

      put("22", R.drawable.alphC);
      put("22n", R.drawable.alphCn);
      put("22e", R.drawable.alphCe);
      put("22s", R.drawable.alphCs);
      put("22w", R.drawable.alphCw);

      put("23", R.drawable.alphD);
      put("23n", R.drawable.alphDn);
      put("23e", R.drawable.alphDe);
      put("23s", R.drawable.alphDs);
      put("23w", R.drawable.alphDw);

      put("24", R.drawable.alphE);
      put("24n", R.drawable.alphEn);
      put("24e", R.drawable.alphEe);
      put("24s", R.drawable.alphEs);
      put("24w", R.drawable.alphEw);

      put("25", R.drawable.alphF);
      put("25n", R.drawable.alphFn);
      put("25e", R.drawable.alphFe);
      put("25s", R.drawable.alphFs);
      put("25w", R.drawable.alphFw);

      put("26", R.drawable.alphG);
      put("26n", R.drawable.alphGn);
      put("26e", R.drawable.alphGe);
      put("26s", R.drawable.alphGs);
      put("26w", R.drawable.alphGw);

      put("27", R.drawable.alphH);
      put("27n", R.drawable.alphHn);
      put("27e", R.drawable.alphHe);
      put("27s", R.drawable.alphHs);
      put("27w", R.drawable.alphHw);

      put("28", R.drawable.alphS);
      put("28n", R.drawable.alphSn);
      put("28e", R.drawable.alphSe);
      put("28s", R.drawable.alphSs);
      put("28w", R.drawable.alphSw);

      put("29", R.drawable.alpht);
      put("29n", R.drawable.alphtn);
      put("29e", R.drawable.alphte);
      put("29s", R.drawable.alphts);
      put("29w", R.drawable.alphtw);

      put("30", R.drawable.alphU);
      put("30n", R.drawable.alphUn);
      put("30e", R.drawable.alphUe);
      put("30s", R.drawable.alphUs);
      put("30w", R.drawable.alphUw);

      put("31", R.drawable.alphV);
      put("31n", R.drawable.alphVn);
      put("31e", R.drawable.alphVe);
      put("31s", R.drawable.alphVs);
      put("31w", R.drawable.alphVw);

      put("32", R.drawable.alphW);
      put("32n", R.drawable.alphWn);
      put("32e", R.drawable.alphWe);
      put("32s", R.drawable.alphWs);
      put("32w", R.drawable.alphWw);

      put("33", R.drawable.alphX);
      put("33n", R.drawable.alphXn);
      put("33e", R.drawable.alphXe);
      put("33s", R.drawable.alphXs);
      put("33w", R.drawable.alphXw);

      put("34", R.drawable.alphY);
      put("34n", R.drawable.alphYn);
      put("34e", R.drawable.alphYe);
      put("34s", R.drawable.alphYs);
      put("34w", R.drawable.alphYw);

      put("35", R.drawable.alphZ);
      put("35n", R.drawable.alphZn);
      put("35e", R.drawable.alphZe);
      put("35s", R.drawable.alphZs);
      put("35w", R.drawable.alphZw);

      put("36", R.drawable.arrowUP);
      put("36n", R.drawable.arrowUP);
      put("36s", R.drawable.arrowUP);
      put("36e", R.drawable.arrowUP);
      put("36w", R.drawable.arrowUP);

      put("37", R.drawable.arrowUPs);
      put("37n", R.drawable.arrowUPs);
      put("37w", R.drawable.arrowUPe);
      put("37e", R.drawable.arrowUPw);
      put("37w", R.drawable.arrowUPn);

      put("38", R.drawable.arrowUPw);
      put("38n", R.drawable.arrowUPw);
      put("38s", R.drawable.arrowUPw);
      put("38e", R.drawable.arrowUPw);
      put("38w", R.drawable.arrowUPw);

      put("39", R.drawable.arrowUPe);
      put("39n", R.drawable.arrowUPe);
      put("39e", R.drawable.arrowUPe);
      put("39w", R.drawable.arrowUPe);
      put("39s", R.drawable.arrowUPe);

      put("40", R.drawable.circle);
      put("40n", R.drawable.circle);
      put("40s", R.drawable.circle);
      put("40e", R.drawable.circle);
      put("40w", R.drawable.circle);

      put("10", R.drawable.bullseye);
    }
  };
}
